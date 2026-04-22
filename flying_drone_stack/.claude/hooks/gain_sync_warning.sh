#!/bin/bash
# PostToolUse hook: fires after editing firmware_app/src/lib.rs.
# Warns if gain constants (KP/KV/KR/KW/KI) appear in the diff — the identical
# gains also live in src/controller/mod.rs and must be kept in sync manually.

INPUT=$(cat)
FILE=$(echo "$INPUT" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('tool_input',{}).get('file_path',''))" 2>/dev/null)

# Only watch firmware_app/src/lib.rs
if [[ "$FILE" != *firmware_app/src/lib.rs ]]; then
    exit 0
fi

# Check if the edit touched gain constants
NEW=$(echo "$INPUT" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('tool_input',{}).get('new_string',''))" 2>/dev/null)
if echo "$NEW" | grep -qE 'const K(P|V|R|W|I)'; then
    echo ""
    echo "⚠️  GAIN SYNC REMINDER"
    echo "You changed gain constants in firmware_app/src/lib.rs."
    echo "The offboard GeometricController in src/controller/mod.rs has its own"
    echo "copy of these gains — update it too if the change is intentional."
    echo "(The two copies are independent: firmware is no_std, controller/ uses std.)"
fi
exit 0
