# IndexError debug checklist — `robots.<name>.initial_position/uri`

Run in order. Every step is safe — nothing here arms or flies the drone. Stop as soon
as one step gives a real answer; you don't need to run everything if #2 or #3 already
show the crash reliably or not at all.

| # | Step | Command | What it tells you |
|---|------|---------|--------------------|
| 1 | No duplicate server | `ps aux \| grep crazyflie_server`<br>`ros2 node list \| grep -i crazyflie` | More than one process/node = two servers fighting over the same service names. Kill extras (`pkill -f crazyflie_server`) and relaunch clean before continuing. |
| 2 | Version sanity | `python3 -c "import rclpy; print(rclpy.__file__)"`<br>`echo $RMW_IMPLEMENTATION`<br>`ros2 doctor --report \| grep -i rclpy` | Confirms the venv's `rclpy` is the one built for this ROS distro, not a stray pip install shadowing it. A mismatch can change how empty/partial service responses behave. |
| 3 | Reproduction rate | `python3 debug/connect_probe.py --attempts 10 --pause 3` | Loops `Crazyswarm()` alone 10x, no flight. Gives `N/10 passed`. 0 failures once the server's warm = timing-sensitive or already fixed. Frequent/every-time failure = NOT a narrow race, something structural — go to step 5. |
| 4 | Exact timing | `python3 debug/param_race_timeline.py --cfname cf231_active` | Start it right as you launch the server. Prints the moment each of 3 signals goes true and the gap between "robot discoverable" and "params gettable." A real, non-trivial gap (>20ms) = race theory confirmed with numbers, not guessing. Near-zero gap = race theory is wrong, look elsewhere. |
| 5 | Direct service probe | `ros2 param get /crazyflie_server robots.cf231_active.initial_position`<br>`ros2 param get /crazyflie_server robots.cf231_active.uri` | Same service call that crashes, run by hand right before you'd run a flight script. If these come back with real values every time and `flight.py` *still* crashes right after, the bug isn't about timing at all — it's something in how `crazyflie_py`'s own call is built (worth reading its exact request construction again with fresh eyes). |
| 6 | Timestamped server log | `ros2 launch crazyflie launch.py 2>&1 \| ts '[%H:%M:%.S]' \| tee server_launch_$(date +%Y%m%d_%H%M%S).log` (needs `moreutils`; without it: `awk '{ print strftime("[%H:%M:%S]"), $0; fflush() }'`) | Only needed if 1–5 didn't produce a clear answer. Lets you line up the server's own connection/log messages against the probe's timeline, instead of trusting "I saw it connect" by eye. |

## After you have data

- **Consistent failure (steps 3/4 fail often, gap is real):** the race theory holds — fix belongs in `crazyflie_py`'s `Crazyflie.__init__` (add a bounded retry around the `get_parameters` call). That's a deliberate, tested change to shared code, not a lab-day patch — bring the numbers back and we do it properly.
- **Never fails once warmed up (step 3 = 10/10 pass):** not a bug at all in the sense we've been chasing — treat it as "wait a beat after launching the server" and move on with C.0.
- **Fails even with params confirmed present by hand (step 5):** the mechanism is different from anything tested so far — bring the exact terminal output, that's the next real lead, not a guess.

## Files
- `debug/connect_probe.py` — step 3
- `debug/param_race_timeline.py` — step 4
