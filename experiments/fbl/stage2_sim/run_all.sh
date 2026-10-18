#!/bin/bash
# Stage 2 full matrix: ideal measurements and hardware-like (position/gyro noise from the hardware logs, 20 ms position delay assumed)
PY=~/.pyenv/versions/flying_robots/bin/python
cd "$(dirname "$0")"; T=${1:-v1}
$PY s2_closed_loop.py --tag ${T}_ideal 2>&1 | grep "^truth" > ../out/stage2/log_${T}_ideal.txt
$PY s2_closed_loop.py --tag ${T}_hw --noise both --delay-ms 20 2>&1 | grep "^truth" > ../out/stage2/log_${T}_hw.txt
touch ../out/stage2/${T}_done
