"""Run one flight: the real firmware controller in the calibrated plant, from takeoff to landing, and return the result.

Per 1 ms tick: reference and top vehicle -> noisy sensors -> firmware controller (every 2 ms for S0 - S3, every 1 ms for FBL) -> mixer and 4 ms command
delay -> motor lag and thrust law -> downwash force and plume torque (every 2 ms) -> rigid-body step. The order of the random draws is fixed (sensors.py),
so a given (controller, scenario cell, gains, variant, seed) always gives the same numbers.
"""
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from . import config, controllers, downwash, firmware, sensors, variants, vehicle
from .flightplan import FlightPlan
from .scenarios import Motion, study_cells

_TRUTH = {}


def _ns2():
    if 'net' not in _TRUTH:
        _TRUTH['net'] = downwash.Ns2Truth()
    return _TRUTH['net']


@dataclass
class EpisodeConfig:
    """What to simulate. `cell` = settings of the scenario (see frsim.scenarios.cells), None = the study cell of the scenario."""
    controller: str
    scenario: str
    cell: Optional[dict] = None
    gains: Optional[dict] = None              # names of the controller; missing ones take the default
    variant: object = 'nominal'               # name or frsim.variants.Variant
    seed: int = 0
    plant: config.Plant = config.CALIBRATED_PLANT
    stop_on_crash: bool = False               # stop at the first tilt >= crash limit (faster; the RMS fields of a crashed episode are then not valid)


@dataclass
class EpisodeResult:
    stable: bool                  # False = crashed (tilt limit reached inside the scoring window), error counts as 100 cm
    completed: bool               # False = the run was stopped early (crash stop or numerical failure)
    x_rms_cm: float
    y_rms_cm: float
    z_rms_cm: float
    tilt_max_deg: float
    xy_steady_cm: float
    yaw_absmax_deg: float
    tilt_rms_deg: float
    fbl_active_pct: float
    crash_time_s: Optional[float]  # scenario time of the first sample at / above the crash tilt, None if none
    n_samples: int
    scenario_duration_s: float
    trace: np.ndarray = field(repr=False, default=None)    # columns: scenario time (s, negative = takeoff / hold), error x / y / z (m), yaw, tilt (deg), fbl active, fbl tripped, position x / y / z (m); every 10 ms


def _set_xyz(target, value):
    target.x, target.y, target.z = map(float, value)


def run_episode(cfg):
    """Simulate one episode and return an EpisodeResult."""
    fw = firmware.load()
    controller = controllers.get(cfg.controller)
    gains = controllers.check_gains(controller, cfg.gains or {})
    variant = variants.get(cfg.variant)
    plant = cfg.plant
    cell = cfg.cell if cfg.cell is not None else study_cells(cfg.scenario)[0]
    motion = Motion(cfg.scenario, cell)
    plan = FlightPlan(motion)
    mass, g, dt = config.MASS_KG, config.GRAVITY, config.DT_S
    crash_tilt = config.CRASH_TILT_DEG['tree' if controller.tree else 'fbl']
    net = _ns2()

    omar = fw.controllerOmarIndi_t()
    fw.controllerOmarIndiInit(omar)
    controllers.configure(fw, omar, controller, gains)
    setpoint, sens, state, control = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    setpoint.mode.x = setpoint.mode.y = setpoint.mode.z = fw.modeAbs
    setpoint.mode.yaw = fw.modeAbs
    setpoint.attitude.yaw = 0.0

    sen = sensors.Sensors(cfg.seed, plant, variant, controller.tree)
    body = vehicle.RigidBody(plan.start_position[:2])
    lag_ms = plant.motor_lag_ms if variant.motor_lag_ms is None else variant.motor_lag_ms
    motor_alpha = dt / (lag_ms * 1e-3 + dt)
    command_queue = []
    commanded = body.forces.copy()
    external_force = np.zeros(3)
    plume = np.zeros(3)
    n_ticks = int(plan.total_s / dt)
    scenario_s = plan.scenario_s
    rows = []

    for i in range(n_ticks):
        t = i * dt
        ts = plan.scenario_time(t)
        top_position, top_velocity = plan.top_vehicle(t)
        if i % config.PEER_EVERY_TICKS == 0:
            fw.oot_set_peer(0, float(top_position[0]), float(top_position[1]), float(top_position[2]), int(i))
        pr, vr, ar, jr = plan.setpoint(t)
        _set_xyz(setpoint.position, pr)
        _set_xyz(setpoint.velocity, vr)
        if variant.feedforward_scale is not None:
            ar = ar * variant.feedforward_scale
            jr = jr * variant.feedforward_scale
        _set_xyz(setpoint.acceleration, ar)
        _set_xyz(setpoint.jerk, jr)

        measured_x, measured_v = sen.state(body.x, body.v)
        _set_xyz(state.position, measured_x)
        _set_xyz(state.velocity, measured_v)
        qx, qy, qz, qw = map(float, body.q)
        state.attitudeQuaternion.x, state.attitudeQuaternion.y, state.attitudeQuaternion.z, state.attitudeQuaternion.w = qx, qy, qz, qw
        attitude = Rotation.from_quat(body.q)
        rotation = attitude.as_matrix()
        euler = attitude.as_euler('xyz', degrees=True)          # roll, pitch, yaw (deg); the firmware state uses the legacy pitch sign
        state.attitude.roll, state.attitude.pitch, state.attitude.yaw = float(euler[0]), float(-euler[1]), float(euler[2])
        _set_xyz(sens.gyro, sen.gyro(body.w))

        if controller.tree:
            specific_force = sen.accelerometer(rotation.T @ (body.previous_acceleration + np.array([0, 0, g])) / g)
            _set_xyz(sens.acc, specific_force)
            _set_xyz(state.acc, rotation @ specific_force - np.array([0, 0, 1.0]))      # world frame, gravity excluded [g]
            fw.oot_set_rpm(*sen.rpm(body.forces))
        if (not controller.tree) or i % 2 == 0:                  # S0 - S3 at 500 Hz like the firmware, FBL at 1 kHz
            fw.controllerOmarIndi(omar, control, setpoint, sens, state, i)

        if i % 2 == 0:
            forces = vehicle.mixer_forces(fw, control)
            if np.isfinite(forces).all():
                command_queue.append((t + config.COMMAND_DELAY_S, forces))
        while command_queue and command_queue[0][0] <= t:
            _, commanded = command_queue.pop(0)
        target_force = vehicle.real_thrust(commanded, plant.max_motor_force_n)
        body.forces = body.forces + motor_alpha * (target_force - body.forces)
        thrust, torque = vehicle.wrench(body.forces)
        torque = torque + 1.0 * plume

        if i % 2 == 0:                                           # downwash of the top vehicle (only when it is above)
            above = top_position[2] > body.x[2]
            neighbours = [(top_position - body.x, top_velocity - body.v)] if above else []
            a_nn = net.acceleration(neighbours, float(body.x[2]), body.v, mass)
            external_force = np.asarray(a_nn, float) * mass * plant.downwash_scale
            plume = np.zeros(3)
            if above:
                plume += downwash.plume_torque(body.x, rotation, top_position, plant)

        body.step(rotation, attitude, thrust, torque, external_force, variant.lateral_thrust_scale, dt)
        tilt = np.degrees(np.arccos(np.clip(rotation[2, 2], -1, 1)))
        if i % 10 == 0:
            rows.append([t, *(body.x - pr), euler[2], tilt, omar.fbl.active, omar.fbl.tripped, *body.x])
            if cfg.stop_on_crash and 0 < ts <= scenario_s and tilt >= crash_tilt:
                break
        if not np.isfinite(body.x).all() or tilt > 120:
            break

    return _summarize(np.array(rows), n_ticks, scenario_s, controller.tree, crash_tilt)


def _summarize(r, n_ticks, scenario_s, tree, crash_tilt):
    ts = r[:, 0] - config.TAKEOFF_S - config.HOLD_S
    window = (ts > config.SCORE_AFTER_S) & (ts <= scenario_s)
    late = (ts > (scenario_s / 2)) & (ts <= scenario_s)
    during = (ts > 0) & (ts <= scenario_s)

    def rms(a):
        return 100 * float(np.sqrt(np.mean(a ** 2))) if a.size else float('nan')

    completed = len(r) * 10 >= n_ticks - 10
    tilt_max = float(r[during, 5].max()) if (ts > 0).any() else float('nan')
    active_pct = 100 * float(r[during, 6].mean()) if (ts > 0).any() else 0.0
    tripped = bool(r[:, 7].max())
    if tree:
        stable = bool(completed and tilt_max < crash_tilt)
    else:
        stable = bool(completed and not tripped and tilt_max < crash_tilt and active_pct > 99)
    crash_idx = np.nonzero(during & (r[:, 5] >= crash_tilt))[0]
    return EpisodeResult(
        stable=stable, completed=bool(completed), x_rms_cm=rms(r[window, 1]), y_rms_cm=rms(r[window, 2]), z_rms_cm=rms(r[window, 3]),
        tilt_max_deg=tilt_max, xy_steady_cm=100 * float(np.hypot(r[late, 1].mean(), r[late, 2].mean())) if late.any() else float('nan'),
        yaw_absmax_deg=float(np.abs(r[(ts > 0) & (ts <= scenario_s), 4]).max()) if (ts > 0).any() else float('nan'),
        tilt_rms_deg=float(np.sqrt(np.mean(r[window, 5] ** 2))) if window.any() else float('nan'),
        fbl_active_pct=active_pct, crash_time_s=(float(ts[crash_idx[0]]) if len(crash_idx) else None), n_samples=len(r),
        scenario_duration_s=float(scenario_s), trace=np.column_stack([ts, r[:, 1:]]))
