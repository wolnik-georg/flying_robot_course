#!/usr/bin/env bash
# Find the AI Deck's IP address on the home network.
# Scans for port 5000, then verifies each hit is actually an AI Deck
# (not e.g. macOS AirPlay which also uses port 5000).
#
# Usage:
#   ./scripts/find_aideck.sh [subnet]
#
# Requires: nmap, python3

set -e

SUBNET="${1:-}"

if [ -z "$SUBNET" ]; then
    GW_IF=$(ip route | awk '/^default/{print $5; exit}')
    if [ -z "$GW_IF" ]; then
        echo "Could not autodetect network interface. Pass subnet manually:"
        echo "  ./scripts/find_aideck.sh 192.168.178.0/24"
        exit 1
    fi
    SUBNET=$(ip -4 addr show "$GW_IF" | awk '/inet /{print $2}' | head -1)
    echo "Detected subnet: $SUBNET (interface $GW_IF)"
fi

echo ""
echo "Scanning $SUBNET for port 5000 ..."

if ! command -v nmap &>/dev/null; then
    echo "nmap not found:  sudo apt install nmap"
    exit 1
fi

# Find all hosts with port 5000 open
CANDIDATES=$(nmap -p 5000 --open -T4 "$SUBNET" 2>/dev/null \
    | awk '/Nmap scan report/{ip=$NF} /5000\/tcp.*open/{print ip}' \
    | tr -d '()')

if [ -z "$CANDIDATES" ]; then
    echo ""
    echo "No host found with port 5000 open."
    echo ""
    echo "Possible reasons:"
    echo "  - AI Deck not powered on, or still booting (wait ~15 s)"
    echo "  - AI Deck still in AP mode — hasn't been flashed yet"
    echo "  - AI Deck connected to a different subnet"
    echo ""
    echo "To flash the STM32 firmware for STA mode:"
    echo "  ./scripts/flash_crazyflie.sh"
    exit 1
fi

echo ""
echo "Verifying candidates (filtering out macOS AirPlay, dev servers, etc.)..."

AI_DECK_IP=""

# Python one-liner: try to connect and read the first CPX packet header.
# A real AI Deck sends CPX packets: [u16-LE wireLength][targets][function][data...]
# The first byte of a WiFi-connected AI Deck is typically the routing byte
# with source=GAP8(4) or ESP32(2). We just check: does it respond within 3 s
# with at least 4 bytes that look like a valid CPX wireLength (2-1024)?

for IP in $CANDIDATES; do
    echo -n "  $IP ... "
    RESULT=$(python3 - "$IP" <<'PYEOF'
import socket, struct, sys
ip = sys.argv[1]
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3.0)
    s.connect((ip, 5000))
    data = s.recv(4)
    s.close()
    if len(data) >= 2:
        wire_len = struct.unpack_from('<H', data, 0)[0]
        # CPX wireLength is typically 3-50 bytes; AirPlay sends HTTP/TLS
        if 2 <= wire_len <= 200:
            print("AI_DECK")
        else:
            print("NOT_AI_DECK")
    else:
        print("NO_DATA")
except Exception as e:
    print(f"TIMEOUT:{e}")
PYEOF
)
    if [ "$RESULT" = "AI_DECK" ]; then
        echo "AI Deck confirmed"
        AI_DECK_IP="$IP"
        break
    else
        echo "not AI Deck ($RESULT)"
    fi
done

echo ""

if [ -z "$AI_DECK_IP" ]; then
    echo "No AI Deck found on the network."
    echo ""
    echo "Is the Crazyflie + AI Deck powered on and the STM32 firmware flashed?"
    echo "  ./scripts/flash_crazyflie.sh   # flash once"
    echo "  Power cycle, wait 15 s, then retry."
    exit 1
fi

echo "AI Deck found at: $AI_DECK_IP"
echo ""
echo "Test with:"
echo "  cargo run --release --bin ai_deck_test -- --addr ${AI_DECK_IP}:5000"
