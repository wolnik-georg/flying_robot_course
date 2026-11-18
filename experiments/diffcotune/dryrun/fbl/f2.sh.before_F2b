#!/bin/bash
# Stage F2.6 queue (heavy, one job at a time through job_runner.py). Before EVERY job: the lab-day guard (F2 prompt section 2): while ~/Desktop/LAB_SESSION_ACTIVE exists or a round.py / session.py process runs,
# wait (poll every 5 min). A regression that is not 0.0 stops the queue. The bracket pattern keeps pgrep from matching this script's own command line.
cd "$(dirname "$0")"
guard() { while [ -e "$HOME/Desktop/LAB_SESSION_ACTIVE" ] || pgrep -f "[r]ound\.py|[s]ession\.py" > /dev/null; do echo "$(date +%T) lab guard holds: waiting before $1"; sleep 300; done; }
echo "$(date +%T) f2.sh start"
for j in $(cat f2_jobs.txt); do
  guard $j
  echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; rc=$(cat logs/$j.rc 2>/dev/null); echo "$(date +%T) done $j rc $rc"
  case $j in
    regressF2_*) if [ "$rc" != "0" ]; then echo "$(date +%T) REGRESSION NOT 0.0 or job failed: queue stopped"; exit 1; fi ;;
  esac
done
echo "$(date +%T) ALL DONE"
