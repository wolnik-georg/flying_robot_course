#!/bin/bash
# Stage F2.5: S0-S3 regression of the lab tools after the FBL edits (round.py, session.py, preflight.py, lab_pool.py, usd_ingest.py): the existing mock modes and preflight, unchanged tests
cd "$(dirname "$0")/.."
while [ ! -f results/mocks_F2_fbl_done.txt ]; do sleep 20; done
for m in "" rs ext regress t3b; do
  f=results/test_r3_mock_F2_${m:-a}.txt
  OMP_NUM_THREADS=2 nice -n 19 python3 test_r3_mock.py ${m:+--$m} > $f 2>&1; echo "rc $?" >> $f
done
for c in S0 S1 S2 S3; do OMP_NUM_THREADS=1 nice -n 19 python3 preflight.py --controller $c > results/preflight_${c}_F2.txt 2>&1; echo "rc $?" >> results/preflight_${c}_F2.txt; done
echo done > results/mocks_F2_s0_done.txt
