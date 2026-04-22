#!/bin/bash
# PostToolUse hook: runs after any Edit/Write on Rust files.
# Runs cargo check and surfaces errors immediately so Claude can fix them
# without needing an explicit "cargo build" call.

INPUT=$(cat)
FILE=$(echo "$INPUT" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('tool_input',{}).get('file_path',''))" 2>/dev/null)

# Only act on .rs files
if [[ "$FILE" != *.rs ]]; then
    exit 0
fi

# Firmware app gets a different target
if [[ "$FILE" == *firmware_app* ]]; then
    OUTPUT=$(cd /home/georg/Desktop/flying_robot_course/flying_drone_stack/firmware_app && \
        LIBCLANG_PATH=/usr/lib/llvm-14/lib \
        cargo check --target thumbv7em-none-eabihf --quiet 2>&1)
    EXIT=$?
    if [ $EXIT -ne 0 ]; then
        echo "⚠️  firmware_app cargo check FAILED:"
        echo "$OUTPUT"
    fi
    exit 0
fi

# Main crate
OUTPUT=$(cd /home/georg/Desktop/flying_robot_course/flying_drone_stack && \
    cargo check --quiet 2>&1)
EXIT=$?
if [ $EXIT -ne 0 ]; then
    echo "⚠️  cargo check FAILED:"
    echo "$OUTPUT"
fi
# On success: no output (don't clutter)
exit 0
