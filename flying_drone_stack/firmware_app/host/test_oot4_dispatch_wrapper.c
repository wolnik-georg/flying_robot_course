/* Standalone test of the ACTUAL real-firmware dispatch path for controller=9:
 * controllerOutOfTree4Init/Test/controllerOutOfTree4, which are gated behind
 * #ifdef CRAZYFLIE_FW and therefore never compiled into the host/SWIG build (confirmed:
 * hasattr(cffirmware, 'controllerOutOfTree4Init') is False there). Every other test
 * (test_omar_indi_reference.py, the CS2 SIL) calls the explicit-self core functions
 * (controllerOmarIndiInit/controllerOmarIndi) directly, never this thin wrapper around the
 * static g_self that real hardware's controller.c dispatch table actually calls.
 *
 * Compiles controller_omar_indi.c a second time, standalone, WITH -DCRAZYFLIE_FW, and calls the
 * wrapper functions directly -- the only way to exercise this exact code path without a real
 * board. Run via test_oot4_dispatch_wrapper.sh.
 *
 * g_self's struct initializer defaults to indi=0 (geometric only, no RPM feedback) -- the
 * wrapper cannot be told otherwise from outside without the real PARAM subsystem (not linked
 * here), so this exercises the geometric-only path. Expected result for the exact-hover case
 * below: thrustSi = CF_MASS * GRAVITY = 0.0393 * 9.81 = 0.385533 (pure gravity feedforward,
 * zero position/attitude error) -- confirmed 2026-09-23. */
#include <stdio.h>
#include <string.h>
#include "math3d.h"
#include "controller_omar_indi.h"

int main(void) {
    controllerOutOfTree4Init();
    if (!controllerOutOfTree4Test()) {
        printf("FAIL: controllerOutOfTree4Test() returned false\n");
        return 1;
    }

    setpoint_t sp;
    memset(&sp, 0, sizeof(sp));
    sp.position.x = 0; sp.position.y = 0; sp.position.z = 0.5f;
    sp.mode.x = modeAbs; sp.mode.y = modeAbs; sp.mode.z = modeAbs;
    sp.mode.yaw = modeAbs;

    sensorData_t sensors;
    memset(&sensors, 0, sizeof(sensors));

    state_t state;
    memset(&state, 0, sizeof(state));
    state.position.x = 0; state.position.y = 0; state.position.z = 0.5f;
    state.attitudeQuaternion.w = 1.0f;

    control_t control;
    memset(&control, 0, sizeof(control));

    for (uint32_t tick = 0; tick < 40; tick++) {
        controllerOutOfTree4(&control, &sp, &sensors, &state, tick);
    }

    printf("thrustSi=%.6f torque=(%.6f,%.6f,%.6f)\n",
           control.thrustSi, control.torque[0], control.torque[1], control.torque[2]);

    const float expected = 0.385533f;
    if (control.thrustSi != control.thrustSi /* NaN check */
            || control.thrustSi < expected - 1e-3f || control.thrustSi > expected + 1e-3f) {
        printf("FAIL: thrustSi=%.6f, expected ~%.6f (CF_MASS * GRAVITY)\n",
               control.thrustSi, expected);
        return 1;
    }
    printf("PASS: wrapper dispatch executed, output matches CF_MASS * GRAVITY exactly\n");
    return 0;
}
