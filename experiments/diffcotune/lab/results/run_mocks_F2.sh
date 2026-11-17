#!/bin/bash
cd "$(dirname "$0")/.."
for m in "" rs t3b; do
  f=results/test_r3_mock_F2_fbl_${m:-a}.txt
  OMP_NUM_THREADS=2 nice -n 19 python3 test_r3_mock_fbl.py ${m:+--$m} > $f 2>&1; echo "rc $?" >> $f
done
echo done > results/mocks_F2_fbl_done.txt
