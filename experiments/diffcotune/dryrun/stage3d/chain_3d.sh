#!/bin/bash
# Stage 3d chain: waits for the gradient diagnostic (exact PID given as $1) to end, then runs the jobs one after the other under the standing limits
# (MemAvailable >= 3 GB and load <= 22 before each start, load logged one minute after each start, nice -n 19, 2 torch threads per worker via loop_lib).
cd /home/georg/Desktop/flying_robot_course/experiments/diffcotune
while ps -p "$1" >/dev/null; do sleep 15; done
L=dryrun/stage3d/logs
gate() { while true; do m=$(LC_ALL=C awk '/MemAvailable/{print int($2/1048576*10)/10}' /proc/meminfo); l=$(awk '{print int($1)}' /proc/loadavg); if [ "$(echo "$m >= 3.0" | bc)" = 1 ] && [ "$l" -le 22 ]; then break; fi; sleep 20; done; }
run() { name=$1; shift; gate; echo "$(date +%T) start $name" >> $L/chain.log
  nice -n 19 "$@" > $L/$name.log 2>&1 & pid=$!; echo "$(date +%T) $name pid $pid" >> $L/chain.log
  sleep 60; echo "$(date +%T) $name load after 1 min: $(cut -d' ' -f1-3 /proc/loadavg) MemAvailable $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB" >> $L/chain.log
  wait $pid; echo "$(date +%T) end $name exit $?" >> $L/chain.log; }
run speed_cpu python3 dryrun/stage3d/speed.py cpu
run speed_gpu python3 dryrun/stage3d/speed.py gpu
run confirm python3 dryrun/stage3d/run_3d.py confirm
run variant_A python3 dryrun/stage3d/run_3d.py variant A
run variant_B python3 dryrun/stage3d/run_3d.py variant B
run variant_AB python3 dryrun/stage3d/run_3d.py variant AB
run evalref python3 dryrun/stage3d/run_3d.py evalref
