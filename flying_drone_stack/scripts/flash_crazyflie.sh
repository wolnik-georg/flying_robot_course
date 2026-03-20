#!/usr/bin/env bash
# Flash the pre-built Crazyflie STM32 firmware (configured for AI Deck STA WiFi mode).
#
# The cf2.bin was built with:
#   CONFIG_DECK_AI_WIFI_SETUP_STA=y
#   CONFIG_DECK_AI_SSID="MagentaWLAN-HGDJ"
#   CONFIG_DECK_AI_PASSWORD="67777780147085765540"
#
# After flashing, when the Crazyflie powers on with the AI Deck attached,
# the STM32 sends WiFi config to the Nina W102 so it connects to MagentaWLAN-HGDJ
# instead of creating its own AP.
#
# Usage:
#   ./scripts/flash_crazyflie.sh [URI]
#
# Default URI: radio://0/80/2M/E7E7E7E7E7
# Crazyradio must be plugged in. Crazyflie must be powered on.

set -e

URI="${1:-radio://0/80/2M/E7E7E7E7E7}"
BIN="/home/georg/Desktop/crazyflie-firmware/build/cf2.bin"

if [ ! -f "$BIN" ]; then
    echo "ERROR: cf2.bin not found at $BIN"
    exit 1
fi

echo "============================================"
echo "  Crazyflie STM32 firmware flash"
echo "============================================"
echo "  URI : $URI"
echo "  BIN : $BIN"
echo "  size: $(stat -c%s "$BIN") bytes"
echo ""
echo "  WiFi config in this firmware:"
echo "    Mode    : STA (connects to home network)"
echo "    SSID    : MagentaWLAN-HGDJ"
echo ""
echo "  Make sure Crazyflie is powered on and Crazyradio is plugged in."
echo ""
read -rp "  Press Enter to flash, Ctrl-C to abort..."

echo ""
echo "Flashing (warm boot — Crazyflie must be powered on and connected via radio)..."
cfloader -w "$URI" flash "$BIN" stm32-fw

echo ""
echo "Flash complete."
echo ""
echo "Next steps:"
echo "  1. Power-cycle the Crazyflie + AI Deck"
echo "  2. Wait ~10 seconds for AI Deck to boot and connect to MagentaWLAN-HGDJ"
echo "  3. Find the AI Deck's IP:  ./scripts/find_aideck.sh"
echo "  4. Test camera:  cargo run --release --bin ai_deck_test -- --addr <IP>:5000"
echo ""
echo "If the AI Deck still creates an AP (GAP8 overrides STM32 config):"
echo "  Run  ./scripts/configure_aideck_sta.py  while connected to the AP WiFi."
