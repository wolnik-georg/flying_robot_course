#!/bin/bash
# Stage F2.6 queue (heavy, one job at a time through job_runner.py), F2b version. Before EVERY job: the lab-day guard (wait while ~/Desktop/LAB_SESSION_ACTIVE exists or a round.py / session.py process runs, poll every 5 min).
# F2b changes: (1) MemAvailable is checked every 5 min during a job; below 3 GB the job is stopped by the exact PIDs of its process tree (not pkill) and the queue stops; (2) a job with rc != 0 (a FAIL of any gate or regression, or a failed run) stops the queue.
# The bracket pattern keeps pgrep from matching this script's own command line.
cd "$(dirname "$0")"
guard() { while [ -e "$HOME/Desktop/LAB_SESSION_ACTIVE" ] || pgrep -f "[r]ound\.py|[s]ession\.py" > /dev/null; do echo "$(date +%T) lab guard holds: waiting before $1"; sleep 300; done; }
memgb() { awk '/MemAvailable/{printf "%.1f", $2/1048576}' /proc/meminfo; }
tree() { local p=$1; echo $p; for c in $(pgrep -P $p); do tree $c; done; }
echo "$(date +%T) f2.sh start"
for j in $(cat f2_jobs.txt); do
  guard $j
  echo "$(date +%T) start $j (MemAvailable $(memgb) GB)"
  nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1 &
  rp=$!; low=0
  while kill -0 $rp 2>/dev/null; do
    for i in $(seq 30); do kill -0 $rp 2>/dev/null || break; sleep 10; done
    kill -0 $rp 2>/dev/null || break
    m=$(memgb)
    if awk -v m="$m" 'BEGIN{exit !(m < 3.0)}'; then
      pids=$(tree $rp | tac | tr '\n' ' '); echo "$(date +%T) MemAvailable $m GB < 3 GB during $j: stopping PIDs $pids; queue stopped"
      kill $pids; echo "memory" > logs/$j.rc; exit 2
    fi
  done
  wait $rp; rc=$(cat logs/$j.rc 2>/dev/null); echo "$(date +%T) done $j rc $rc"
  if [ "$rc" != "0" ]; then echo "$(date +%T) $j rc $rc (FAIL of a gate or regression, or a failed job): queue stopped"; exit 1; fi
done
echo "$(date +%T) ALL DONE"
