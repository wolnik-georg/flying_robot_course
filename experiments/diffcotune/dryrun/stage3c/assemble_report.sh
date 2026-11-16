#!/bin/bash
cd "$(dirname "$0")"; python3 report_partb.py > /dev/null; cat report_stage3c_partB.md report_stage3c_partC.md > report_stage3c.md
