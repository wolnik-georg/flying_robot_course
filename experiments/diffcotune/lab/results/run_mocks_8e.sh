#!/bin/bash
cd /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab
for m in rs "" ext regress; do
  f=results/test_r3_mock_8e_${m:-a}.txt
  nice -n 19 python3 test_r3_mock.py ${m:+--$m} > $f 2>&1; echo "rc $?" >> $f
done
echo ALLDONE > results/test_r3_mock_8e_done.txt
