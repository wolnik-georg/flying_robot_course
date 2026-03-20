#!/usr/bin/env python3
"""
Configure AI Deck WiFi to STA (station) mode via the TCP/CPX interface.

Run this while your laptop is connected to the AI Deck's AP WiFi
("WiFi streaming example" or "Bitcraze AI-deck Example").

After running, the Nina W102 will connect to your home WiFi network.
Reconnect your laptop to home WiFi then use find_aideck.sh to discover
the AI Deck's IP.

NOTE: This config is session-only. On next power cycle the GAP8 firmware
may re-assert AP mode. To make it permanent, reflash the STM32 firmware:
  ./scripts/flash_crazyflie.sh

Usage:
  python scripts/configure_aideck_sta.py --ssid <SSID> --password <PASS>
  python scripts/configure_aideck_sta.py  # uses defaults below
"""

import argparse
import socket
import struct
import sys
import time

# --------------------------------------------------------------------------
# CPX constants (from cflib/cpx/__init__.py and wifi.h)
# --------------------------------------------------------------------------
CPX_TARGET_HOST  = 3
CPX_TARGET_ESP32 = 2

CPX_FUNCTION_WIFI_CTRL = 4

WIFI_CTRL_SET_SSID    = 0x10
WIFI_CTRL_SET_KEY     = 0x11
WIFI_CTRL_WIFI_CONNECT = 0x20
WIFI_CONNECT_AS_STA   = 0x00
WIFI_CONNECT_AS_AP    = 0x01

WIFI_STATUS_CONNECTED = 0x31

# --------------------------------------------------------------------------
# Defaults — edit to match your home network
# --------------------------------------------------------------------------
DEFAULT_SSID     = "MagentaWLAN-HGDJ"
DEFAULT_PASSWORD = "67777780147085765540"
DEFAULT_AI_DECK_ADDR = "192.168.4.1"
DEFAULT_AI_DECK_PORT = 5000


def build_cpx_packet(function: int, destination: int, data: bytes, last: bool = True) -> bytes:
    """Encode a CPX packet as it is sent over the TCP socket.

    Wire format (from SocketTransport.writePacket):
      [u16-LE length]  [targets_flags byte]  [function_version byte]  [payload]
    length = 2 (header bytes) + len(payload)
    """
    source = CPX_TARGET_HOST
    targets_and_flags = ((source & 0x7) << 3) | (destination & 0x7)
    if last:
        targets_and_flags |= 0x40          # lastPacket flag

    version = 0
    function_and_version = (function & 0x3F) | ((version & 0x3) << 6)

    header = bytes([targets_and_flags, function_and_version])
    payload = header + data

    wire = struct.pack('<H', len(payload)) + payload
    return wire


def send_wifi_ctrl(sock: socket.socket, cmd: int, data: bytes) -> None:
    """Send a single WiFi control command to the Nina W102 (ESP32)."""
    payload = bytes([cmd]) + data
    pkt = build_cpx_packet(CPX_FUNCTION_WIFI_CTRL, CPX_TARGET_ESP32, payload)
    sock.sendall(pkt)


def wait_for_connected(sock: socket.socket, timeout: float = 15.0) -> str | None:
    """Listen for WIFI_STATUS_CONNECTED from Nina. Returns IP string or None."""
    sock.settimeout(1.0)
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        try:
            chunk = sock.recv(256)
            if not chunk:
                break
            buf += chunk
            # CPX packets: [u16-LE len][targets_flags][func_version][payload...]
            while len(buf) >= 2:
                length = struct.unpack_from('<H', buf, 0)[0]
                if len(buf) < 2 + length:
                    break
                pkt_data = buf[2 : 2 + length]
                buf = buf[2 + length:]
                if len(pkt_data) < 2:
                    continue
                func = pkt_data[1] & 0x3F
                payload = pkt_data[2:]
                if func == CPX_FUNCTION_WIFI_CTRL and len(payload) >= 1:
                    cmd = payload[0]
                    if cmd == WIFI_STATUS_CONNECTED and len(payload) >= 5:
                        ip = "{}.{}.{}.{}".format(*payload[1:5])
                        return ip
        except socket.timeout:
            print(".", end="", flush=True)
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure AI Deck WiFi to STA mode via TCP")
    parser.add_argument("--addr",     default=DEFAULT_AI_DECK_ADDR, help="AI Deck AP IP (default: 192.168.4.1)")
    parser.add_argument("--port",     type=int, default=DEFAULT_AI_DECK_PORT)
    parser.add_argument("--ssid",     default=DEFAULT_SSID,     help="Home WiFi SSID to connect to")
    parser.add_argument("--password", default=DEFAULT_PASSWORD, help="Home WiFi password")
    args = parser.parse_args()

    print("AI Deck WiFi STA configuration")
    print(f"  AI Deck AP address : {args.addr}:{args.port}")
    print(f"  Target SSID        : {args.ssid}")
    print(f"  Password           : {'*' * len(args.password)}")
    print()
    print("Make sure your laptop is connected to the AI Deck's AP WiFi first.")
    print()

    print(f"Connecting to {args.addr}:{args.port} ...")
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(8.0)
        s.connect((args.addr, args.port))
    except Exception as e:
        print(f"ERROR: Could not connect: {e}")
        print()
        print("Is your laptop connected to the AI Deck AP WiFi?")
        print("  - The network is usually named 'WiFi streaming example' or 'Bitcraze AI-deck Example'")
        print("  - Ping 192.168.4.1 to verify connectivity")
        sys.exit(1)

    print("Connected.")
    print()

    print("Sending SSID ...")
    send_wifi_ctrl(s, WIFI_CTRL_SET_SSID, args.ssid.encode())
    time.sleep(0.1)

    print("Sending password ...")
    send_wifi_ctrl(s, WIFI_CTRL_SET_KEY, args.password.encode())
    time.sleep(0.1)

    print("Sending CONNECT (STA mode) ...")
    send_wifi_ctrl(s, WIFI_CTRL_WIFI_CONNECT, bytes([WIFI_CONNECT_AS_STA]))
    time.sleep(0.2)

    print()
    print("Configuration sent. Waiting for Nina to connect to home network", end="", flush=True)
    ip = wait_for_connected(s, timeout=20.0)
    print()

    s.close()

    if ip:
        print(f"AI Deck connected to home WiFi!")
        print(f"  IP address : {ip}")
        print()
        print("Now reconnect your laptop to your home WiFi, then test:")
        print(f"  cargo run --release --bin ai_deck_test -- --addr {ip}:5000")
    else:
        print("Did not receive connection confirmation within 20 s.")
        print("The config was sent — Nina may still be connecting.")
        print()
        print("Find the AI Deck's IP once your laptop is back on home WiFi:")
        print("  ./scripts/find_aideck.sh")
        print()
        print("Or check your router's DHCP client list for a device named 'AI-Deck'")


if __name__ == "__main__":
    main()
