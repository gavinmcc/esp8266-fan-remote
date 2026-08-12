#!/usr/bin/env python3
"""
test_e2e.py — End-to-end test for esp8266-fan-remote

Checks:
  1. TX unit reachable and CC1101 registers correct
  2. Each fan command returns HTTP 200 with expected body
  3. RX unit sees RF signal during each transmission (RSSI spike or pulse capture)

Usage:
  python3 test_e2e.py [--tx-ip IP] [--rx-port PORT]

Defaults:
  --tx-ip   192.168.4.135
  --rx-port auto-detected from /dev/ttyUSB*
"""

import argparse
import glob
import re
import sys
import threading
import time
import urllib.request
import urllib.error


RSSI_SIGNAL_THRESHOLD = -70   # dBm — anything above this during TX counts as "signal seen"
RX_LISTEN_SECONDS     = 2.5   # how long to listen on RX after firing each command (OFF takes ~1.05s)

COMMANDS = [
    ("HI",    "/hi",    "OK: HI"),
    ("MED",   "/med",   "OK: MED"),
    ("LOW",   "/low",   "OK: LOW"),
    ("OFF",   "/off",   "OK: OFF"),
    ("LIGHT", "/light", "OK: LIGHT"),
]

EXPECTED_REGISTERS = {
    "PKTCTRL0": "0x02",
    "MDMCFG2":  "0x30",
    "FREND0":   "0x11",
    "PATABLE[0]": "0x00",
}


def http_get(url, timeout=5):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status, r.read().decode("utf-8", errors="replace")
    except urllib.error.URLError as e:
        return None, str(e)


def auto_detect_rx_port():
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        return None
    if len(ports) == 1:
        return ports[0]
    # Try each port briefly; capture.ino prints [RSSI] lines, fan_remote.ino does not
    for port in ports:
        try:
            import serial
            s = serial.Serial(port, 115200, timeout=0.3)
            data = b""
            end = time.time() + 1.5
            while time.time() < end:
                data += s.read(256)
            s.close()
            if b"[RSSI]" in data:
                return port
        except Exception:
            pass
    return ports[0]


def listen_rx(port, results, duration):
    """Read RX serial for `duration` seconds, record max RSSI and any captures."""
    try:
        import serial
        s = serial.Serial(port, 115200, timeout=0.1)
        max_rssi = -999
        captures = []
        end = time.time() + duration
        while time.time() < end:
            line = s.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            m = re.search(r'\[RSSI\]\s+(-?\d+)', line)
            if m:
                rssi = int(m.group(1))
                if rssi > max_rssi:
                    max_rssi = rssi
            elif line.startswith("@") and "#" in line:
                captures.append(line)
        s.close()
        results["max_rssi"] = max_rssi
        results["captures"] = captures
    except ImportError:
        results["error"] = "pyserial not installed — run: pip install pyserial"
    except Exception as e:
        results["error"] = str(e)


def check_status(tx_base):
    print("\n── Register check ──────────────────────────────────────")
    status, body = http_get(f"{tx_base}/status")
    if status is None:
        print(f"  FAIL  TX unit unreachable: {body}")
        return False

    all_ok = True
    for reg, want in EXPECTED_REGISTERS.items():
        pattern = rf"{re.escape(reg)}\s*=\s*({re.escape(want)})"
        if re.search(pattern, body):
            print(f"  PASS  {reg} = {want}")
        else:
            # Extract what we actually got
            got_m = re.search(rf"{re.escape(reg)}\s*=\s*(0x[0-9A-Fa-f]+)", body)
            got = got_m.group(1) if got_m else "?"
            print(f"  FAIL  {reg}: want {want}, got {got}")
            all_ok = False
    return all_ok


def run_tests(tx_base, rx_port):
    print(f"\nTX: {tx_base}")
    print(f"RX: {rx_port or 'not available — skipping RF checks'}")

    passed = 0
    failed = 0

    if not check_status(tx_base):
        failed += 1
    else:
        passed += 1

    print("\n── Command tests ───────────────────────────────────────")
    for name, path, expected_body in COMMANDS:
        rx_results = {}

        # Start RX listener before firing so we catch the leading edge of TX
        rx_thread = None
        if rx_port:
            rx_thread = threading.Thread(
                target=listen_rx, args=(rx_port, rx_results, RX_LISTEN_SECONDS), daemon=True
            )
            rx_thread.start()
            time.sleep(0.1)  # small head start so listener is ready

        url = f"{tx_base}{path}"
        status, body = http_get(url, timeout=10)

        if rx_thread:
            rx_thread.join()

        # HTTP check
        http_ok = status == 200 and expected_body in body
        http_label = f"HTTP {status}" if status else "HTTP FAIL"

        # RF check
        rf_ok = None
        rf_label = ""
        if "error" in rx_results:
            rf_label = f"RX error: {rx_results['error']}"
        elif rx_results:
            max_rssi = rx_results.get("max_rssi", -999)
            captures = rx_results.get("captures", [])
            rf_ok = max_rssi > RSSI_SIGNAL_THRESHOLD or len(captures) > 0
            if captures:
                rf_label = f"captured {len(captures)} frame(s)"
            else:
                rf_label = f"peak RSSI {max_rssi} dBm ({'signal seen' if rf_ok else 'no signal'})"

        overall = http_ok and (rf_ok is not False)
        status_str = "PASS" if overall else "FAIL"
        if overall:
            passed += 1
        else:
            failed += 1

        http_result = "ok" if http_ok else f"FAIL (status={status}, body={body!r})"
        print(f"  {status_str}  {name:6s}  HTTP={http_result}  RF={rf_label}")

    print(f"\n{'─' * 55}")
    total = passed + failed
    print(f"  {passed}/{total} passed", "✓" if failed == 0 else "✗")
    return failed == 0


def main():
    parser = argparse.ArgumentParser(description="End-to-end test for esp8266-fan-remote")
    parser.add_argument("--tx-ip",   default="192.168.4.135", help="TX unit IP address")
    parser.add_argument("--rx-port", default=None,            help="RX unit serial port (auto-detected if omitted)")
    args = parser.parse_args()

    tx_base = f"http://{args.tx_ip}"
    rx_port = args.rx_port or auto_detect_rx_port()

    ok = run_tests(tx_base, rx_port)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
