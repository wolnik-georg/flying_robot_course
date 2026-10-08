#!/usr/bin/env python3
"""Child process: one two-drone A8 SIL episode where the BOTTOM drone runs OUR full INDI (ctrl_mode 3, ki_z 0, res_sign +1,
pos gains 64/48/5/7 as in the 2026-10-02 hardware flights) and the top drone stays geometric. Same plant as the NS2 canon."""
import json, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import ns2_closed_loop_sil_sim as S  # noqa: E402

_CALLS = [0]

def _mode_for(idx, c, step):
    """Gains are process-global in the host library (oot_select_drone only swaps the controller state), so the
    controller mode / position gains must be re-applied for the drone about to run (cfg per_step_configure=True)."""
    if idx == 0:
        c.g_kp_xy, c.g_kp_z, c.g_kv_xy, c.g_kv_z = 64.0, 48.0, 5.0, 7.0
        c.g_ki_z = 0.0
        c.g_indi_res_sign = 1
        # like the hardware flights: geometric during takeoff and landing, full INDI (ctrl_mode 3) in the scenario
        c.g_controller_mode = 3 if (4000 <= step < 30000 and os.environ.get('OURS_MODE','3')=='3') else 0
    else:
        c.g_kp_xy, c.g_kp_z, c.g_kv_xy, c.g_kv_z = 40.0, 30.0, 8.0, 10.0
        c.g_ki_z = 16.0
        c.g_indi_res_sign = 1
        c.g_controller_mode = 0

def configure_drone(idx, *, rnn_en, div, res_sign, ki_z, use_ref_gains=False):
    S.firm.oot_select_drone(idx)
    c = S.firm.cvar
    if os.environ.get("BOTH") == "1":       # gains are process-global: run BOTH drones as our INDI, configured once
        S.apply_geometric_gains(c, ki_z=0.0, res_sign=1, use_ref_fc=False)
        c.g_indi_fc_bw = 206.0
        c.g_kp_xy, c.g_kp_z, c.g_kv_xy, c.g_kv_z = 64.0, 48.0, 5.0, 7.0
        c.g_ki_z = 0.0
        c.g_controller_mode = 3
        c.g_rnn_div = int(div); c.g_rnn_en = 0
        return
    if _CALLS[0] < 2:                       # initial full configuration (LAB INDI constants, hardware fc_bw 206 for INDI)
        S.apply_geometric_gains(c, ki_z=16.0, res_sign=1, use_ref_fc=False)
        c.g_indi_fc_bw = 206.0
        c.g_rnn_div = int(div)
        c.g_rnn_en = 0
    step = max(0, (_CALLS[0] - 2) // 2)
    _CALLS[0] += 1
    _mode_for(idx, c, step)

S.configure_drone = configure_drone
cfg = json.loads(sys.argv[1])
devnull = os.open(os.devnull, os.O_WRONLY); saved = os.dup(1); os.dup2(devnull, 1)
try:
    result = S.run_episode(cfg)
except Exception as exc:
    result = {"label": cfg.get("label"), "error": str(exc)}
os.write(saved, (json.dumps(result) + "\n").encode())
