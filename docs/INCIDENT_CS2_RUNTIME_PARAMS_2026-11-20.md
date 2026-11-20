# Incident 2026-11-11 → 11-20: CS2 runtime parameter sets silently dead (uSD logs lost)

## In one line each
- **Root cause:** someone reinstalled ROS on the lab PC on 11 Nov. Since then the CS2 server never received its own parameter events, so **no runtime parameter set reached a drone** (uSD start, gains, controller, EKF toggle). Every script still reported success.
- **Solution:** the server now forwards a set the moment it accepts it, instead of waiting for its own event (crazyswarm2 `71219f3`, main + all 11 `study_*` branches).

## Evidence
| What | Result | File |
|---|---|---|
| ROS history on the lab PC | 11 Nov `apt upgrade`, `apt remove ros-*`, reinstall `ros-jazzy-desktop`; 20 Nov GUI updater. Done by someone outside the team. | `debug/lab_logs/rebuild_server_2026-11-20_19*.log` |
| Before the fix | Set accepted and published; server prints no `Update parameter`; drone value unchanged (old and freshly built server) | `usd_diag_2026-11-20_185604.log`, `..._191731.log` |
| Same server, same ROS versions, clean container | Works (receives its own events) → the cause is specific to the lab PC | planning agent, Docker `ros:jazzy` |
| After the fix | all 5 sets forwarded once, drone values change (999 / 997 / 995 both); gate pass OK (thesis02 ≈ 509 kB both); negative OK | `usd_diag_2026-11-20_195200.log`, `debug.log` (d3037b06) |
| Lab PC delivers the server's own events? | **No:** 0 `[diag]` lines with the fix running | `cs2_launch_195100.log` |

## Consequences
- 18 Nov S0 round 0 and the 20 Nov A1/A8/A4 attempts have no uSD files → unusable; re-fly.
- Flights from 11 Nov until the fix ran on yaml (connect-time) values only.
- Flights from before 11 Nov (pilot, Oct) are not affected.

## So it never happens again
1. **Code:** runtime sets no longer depend on DDS delivering a node's own events (`71219f3`). The startup marker is `CS2_RUNTIME_PARAMS_POST_SET_V1`.
2. **Every session, `lab_run.sh prep`:**
   - **stops** if the server binary lacks the fix;
   - **warns** if the ROS packages changed since the last validated session (fingerprint in `~/.ros_jazzy_fingerprint_validated`);
   - then runs the end-to-end uSD gate test (CS2 set → drone writes). That test exercises exactly the runtime-set path.
3. **Every flight:** the uSD gate verifies per drone that the deck writes. No verified write → no takeoff, or land without the trajectory.
4. **Lab PC (admin, once):** freeze ROS and turn off automatic updates.
   - `sudo apt-mark hold $(dpkg-query -W -f='${Package}\n' 'ros-jazzy-*')`
   - Software & Updates → Updates → "Automatically check for updates: Never".
   - Ask whoever administers the PC to announce any ROS change.
5. **Rule:** a script's "started" or "set successful" output is never proof. Only a value read back from the drone counts (log variable / console line).
6. **After any ROS or server change:** run `experiments/diffcotune/lab/usd_cs2_diag.sh` before flying. It must show `server_forwarded=yes` and the drone value changed.

## Tools
- `experiments/diffcotune/lab/apply_runtime_param_fix.sh`: pull + build + marker check.
- `experiments/diffcotune/lab/usd_cs2_diag.sh`: runtime-set path, layer by layer.
- `experiments/diffcotune/lab/param_event_selftest.py`: ROS-only event delivery check.
- `flying_drone_stack/tools/usd_gate_ground_test.py`: pass / negative.
