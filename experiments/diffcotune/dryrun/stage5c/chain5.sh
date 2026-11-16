#!/bin/bash
cd "$(dirname "$0")"
until grep -q "ALL DONE" logs/chain4.log; do sleep 30; done
echo "$(date +%T) start ident2_S1"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_ident2_S1.json > logs/runner_ident2_S1.log 2>&1; echo "$(date +%T) done ident2_S1 rc $(cat logs/ident2_S1.rc 2>/dev/null)"
echo "$(date +%T) ALL DONE"
