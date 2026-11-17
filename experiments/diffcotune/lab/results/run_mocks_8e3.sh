#!/bin/bash
cd /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab
for m in rs "" ext regress; do
  f=results/test_r3_mock_8e3_${m:-a}.txt
  nice -n 19 python3 test_r3_mock.py ${m:+--$m} > $f 2>&1; echo "rc $?" >> $f
done
for c in S0 S1 S2 S3; do nice -n 19 python3 preflight.py --controller $c > results/preflight_${c}_8e3.txt 2>&1; echo "rc $?" >> results/preflight_${c}_8e3.txt; done
echo ALLDONE > results/mocks_8e3_done.txt
