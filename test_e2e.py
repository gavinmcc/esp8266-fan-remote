#!/usr/bin/env python3
"""
test_e2e.py — End-to-end test for esp8266-fan-remote

Checks:
  1. TX unit reachable and CC1101 registers correct
  2. Each fan command returns HTTP 200 with expected body
  3. RX unit sees RF signal during each transmission (RSSI spike or pulse capture)

The RX unit serial port is held open for the full test run. Before each device's
command batch the script sends "FREQ <mhz>" to retune the capture unit.

Usage:
  python3 test_e2e.py [--tx-ip IP] [--rx-port PORT]

Defaults:
  --tx-ip   192.168.4.135
  --rx-port auto-detected from /dev/ttyUSB*
"""

import argparse
import glob
import json
import os
import re
import sys
import threading
import time
import urllib.request
import urllib.error


RSSI_SIGNAL_THRESHOLD = -70   # dBm — anything above this during TX counts as "signal seen"
RX_LISTEN_SECONDS     = 2.5   # how long to listen after firing each command

EXPECTED_REGISTERS = {
    "PKTCTRL0":   "0x02",
    "MDMCFG2":    "0x30",
    "FREND0":     "0x11",
    "PATABLE[0]": "0x00",
}


def load_devices():
    """Return devices list from devices.json."""
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "devices.json")) as f:
        return json.load(f)


def http_get(url, timeout=5):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status, r.read().decode("utf-8", errors="replace")
    except urllib.error.URLError as e:
        return None, str(e)


def parse_pulses(line):
    """Extract [(hi_us, lo_us), ...] from a capture line like '+225-475 +225-250 ...'"""
    return [(int(h), int(l)) for h, l in re.findall(r'\+(\d+)-(\d+)', line)]


def decode_N(pulses, protocol, timing):
    """
    Decode the N value from a captured frame.

    UC7070T:  skip 6-pulse header, count SL before first SS in data section.
    SMC5060RF: count SL before first SS from pulse 0.

    Returns N (int) or None if the frame doesn't parse.
    """
    hi_thresh = (timing['short_h'] + timing['long_h']) / 2 * 25  # midpoint in µs
    lo_thresh = (timing['short_l'] + timing['long_l']) / 2 * 25

    def sym(hi, lo):
        return ('S' if hi < hi_thresh else 'L') + ('S' if lo < lo_thresh else 'L')

    start = 6 if protocol == 'UC7070T' else 0
    if len(pulses) <= start:
        return None

    N = 0
    for hi, lo in pulses[start:]:
        s = sym(hi, lo)
        if s == 'SL':
            N += 1
        elif s == 'SS':
            return N
        else:
            return None  # unexpected symbol before SS
    return None


def auto_detect_rx_port():
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        return None
    if len(ports) == 1:
        return ports[0]
    for port in ports:
        try:
            import serial
            s    = serial.Serial(port, 115200, timeout=0.3)
            data = b""
            end  = time.time() + 1.5
            while time.time() < end:
                data += s.read(256)
            s.close()
            if b"[RSSI]" in data:
                return port
        except Exception:
            pass
    return ports[0]


def rx_set_freq(ser, freq_mhz):
    """Send a FREQ command to the capture unit and wait for it to retune."""
    ser.write(f"FREQ {freq_mhz:.2f}\n".encode())
    time.sleep(0.3)  # CC1101 SetRx takes ~a few ms; 300ms is generous


def listen_rx(ser, results, duration):
    """Read from the already-open serial for duration seconds."""
    max_rssi = -999
    captures = []
    end      = time.time() + duration
    while time.time() < end:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
        except Exception as e:
            results["error"] = str(e)
            return
        if not line:
            continue
        m = re.search(r'\[RSSI\]\s+(-?\d+)', line)
        if m:
            rssi = int(m.group(1))
            if rssi > max_rssi:
                max_rssi = rssi
        elif line.startswith("@") and "#" in line:
            captures.append(line)
    results["max_rssi"] = max_rssi
    results["captures"] = captures


def check_status(tx_base):
    print("\n── Register check ──────────────────────────────────────")
    status, body = http_get(f"{tx_base}/status")
    if status is None:
        print(f"  FAIL  TX unit unreachable: {body}")
        return False

    all_ok = True
    for reg, want in EXPECTED_REGISTERS.items():
        if re.search(rf"{re.escape(reg)}\s*=\s*{re.escape(want)}", body):
            print(f"  PASS  {reg} = {want}")
        else:
            got_m = re.search(rf"{re.escape(reg)}\s*=\s*(0x[0-9A-Fa-f]+)", body)
            got   = got_m.group(1) if got_m else "?"
            print(f"  FAIL  {reg}: want {want}, got {got}")
            all_ok = False
    return all_ok


def run_tests(tx_base, rx_port):
    print(f"\nTX: {tx_base}")
    print(f"RX: {rx_port or 'not available — skipping RF checks'}")

    devices  = load_devices()
    passed   = 0
    failed   = 0

    if not check_status(tx_base):
        failed += 1
    else:
        passed += 1

    # Open RX serial once for the whole run
    rx_ser = None
    if rx_port:
        try:
            import serial
            rx_ser = serial.Serial(rx_port, 115200, timeout=0.1)
            print(f"RX serial open: {rx_port}")
        except ImportError:
            print("pyserial not installed — run: pip install pyserial")
        except Exception as e:
            print(f"RX serial error: {e}")

    try:
        cmd_keys   = ("hi", "med", "low", "off", "light")
        current_freq = None

        print("\n── Command tests ───────────────────────────────────────")
        for dev in devices:
            dev_name  = dev["name"].upper()
            dev_path  = dev["path"]
            freq_mhz  = dev["freq_mhz"]
            short     = dev_name[:3]

            # Retune capture unit when frequency changes
            if rx_ser and freq_mhz != current_freq:
                rx_set_freq(rx_ser, freq_mhz)
                current_freq = freq_mhz
                print(f"  [RX] tuned to {freq_mhz} MHz")

            for key in cmd_keys:
                label    = f"{short}:{key.upper()}"
                endpoint = f"/{dev_path}/{key}"
                expected = f"OK: {dev_name} {key.upper()}"

                rx_results = {}
                rx_thread  = None
                if rx_ser:
                    rx_thread = threading.Thread(
                        target=listen_rx,
                        args=(rx_ser, rx_results, RX_LISTEN_SECONDS),
                        daemon=True,
                    )
                    rx_thread.start()
                    time.sleep(0.1)  # head start so listener is ready

                status, body = http_get(f"{tx_base}{endpoint}", timeout=10)

                if rx_thread:
                    rx_thread.join()

                http_ok = status == 200 and expected in body

                rf_ok    = None
                rf_label = ""
                proto_ok = None
                captures = []
                if "error" in rx_results:
                    rf_label = f"RX error: {rx_results['error']}"
                elif rx_results:
                    max_rssi = rx_results.get("max_rssi", -999)
                    captures = rx_results.get("captures", [])
                    rf_ok    = max_rssi > RSSI_SIGNAL_THRESHOLD or len(captures) > 0
                    if captures:
                        rf_label = f"captured {len(captures)} frame(s)"
                    else:
                        rf_label = f"peak RSSI {max_rssi} dBm ({'signal seen' if rf_ok else 'no signal'})"

                # Protocol validation: decode N from captured frames
                proto_label = ""
                if captures:
                    expected_N = dev['commands'][key]['N']
                    decoded    = [decode_N(parse_pulses(f), dev['protocol'], dev['timing'])
                                  for f in captures]
                    decoded    = [n for n in decoded if n is not None]
                    if not decoded:
                        proto_ok    = False
                        proto_label = "  N=? (decode failed)"
                    else:
                        unique = set(decoded)
                        got_N  = decoded[0] if len(unique) == 1 else f"mixed{sorted(unique)}"
                        if len(unique) == 1 and decoded[0] == expected_N:
                            proto_ok    = True
                            proto_label = f"  N={got_N} ✓"
                        else:
                            proto_ok    = False
                            proto_label = f"  N={got_N} (expected {expected_N}) ✗"

                overall = http_ok and (rf_ok is not False) and (proto_ok is not False)
                if overall:
                    passed += 1
                else:
                    failed += 1

                http_result = "ok" if http_ok else f"FAIL (status={status}, body={body!r})"
                print(f"  {'PASS' if overall else 'FAIL'}  {label:10s}  HTTP={http_result}  RF={rf_label}{proto_label}")

    finally:
        if rx_ser:
            rx_ser.close()

    print(f"\n{'─' * 55}")
    total = passed + failed
    print(f"  {passed}/{total} passed", "✓" if failed == 0 else "✗")
    return failed == 0


def main():
    parser = argparse.ArgumentParser(description="End-to-end test for esp8266-fan-remote")
    parser.add_argument("--tx-ip",   default="192.168.4.135", help="TX unit IP address")
    parser.add_argument("--rx-port", default=None,            help="RX unit serial port (auto-detected if omitted)")
    args = parser.parse_args()

    ok = run_tests(f"http://{args.tx_ip}", args.rx_port or auto_detect_rx_port())
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
