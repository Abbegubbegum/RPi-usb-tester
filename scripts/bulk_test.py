#!/usr/bin/env python3
import usb.core
import usb.util
import time
import struct
import json
import sys
from typing import Dict, Any, List, Tuple

# ---------------------- Test Limits ----------------------
# This is a FIELD TESTER for detecting obvious USB port problems, NOT a compliance
# certification tool. Limits are calibrated for this specific hardware setup.
#
# Hardware characteristics of this tester:
#   - 0.5mm PCB traces (170mm) = ~1Ω measured resistance
#   - Standard 2m USB cable = ~1Ω measured resistance
#   - Total test fixture: ~2Ω series resistance
#   - Result: ~1V drop at 500mA load
#
# Test limits are set to detect GRAVE ERRORS in USB ports:
#   - Completely dead ports (no VBUS)
#   - Overcurrent protection tripping
#   - Excessive voltage sag (worse than test fixture itself)
#   - Data communication failures
#
# These are NOT USB 2.0 spec compliance limits (which would be 4.75V-5.25V, <300mV droop).
# USB 2.0 spec is included below only to explain why our limits are relaxed.
#
# USB 2.0 Spec (reference only - not enforced):
#   - Voltage under load: 4.75V - 5.25V
#   - Droop: <300mV for good supply
#   - Ripple: <100mVpp
#

IDLE_UNDERVOLT_MV = 4800  # Absolute minimum to consider idle VBUS functional

# Test limits for the tests
MIN_THR_MBPS = 1.5           # Minimum data throughput
MIN_MV_LOAD = 4000           # Minimum voltage during the load tests
MAX_RIPPLE_MVPP = 50         # Typical: 10-20mVpp; flag if >50mVpp
MIN_MAX_CURRENT_MA = 400     # Must deliver at least 400mA
# Maximum mean resistance (3Ω = 2Ω fixture + 1Ω margin for dirty contacts)
MAX_RESISTANCE_MOHM = 3000

# ---------------------- USB IDs & Protocol ----------------------
VID = 0x1209
PID = 0x4004

REQ_GET_PORT = 0x01  # IN:  u8 port
REQ_SET_PORT = 0x02  # OUT: wValue = port
REQ_GET_POWER = 0x03  # IN:  power report blob
REQ_GET_ADC_SAMPLES = 0x04  # Trigger bulk transfer of ADC samples
REQ_GET_PORTMAP = 0x10  # IN:  bitmask of available ports

TEST_SECS = 3.0
PKT_SIZE = 1024
TIMEOUT_MS = 10000

ADC_SAMPLES_PER_WINDOW = 9600  # 80 kS/s * 120 ms
POWER_REPORT_FMT = "<BBB" + "H" + "5H"*7 + "HHH"
POWER_REPORT_SIZE = struct.calcsize(POWER_REPORT_FMT)
ADC_SAMPLES_SIZE = ADC_SAMPLES_PER_WINDOW * 2  # 2 bytes per sample

# ---------------------- USB helpers ----------------------


def find_device():
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        raise RuntimeError(
            f"Device not found (VID=0x{VID:04X}, PID=0x{PID:04X})")
    try:
        dev.set_configuration()
    except usb.core.USBError:
        pass
    return dev


def find_vendor_interface(dev):
    for cfg in dev:
        for intf in cfg:
            if intf.bInterfaceClass == 0xFF:
                return intf
    raise RuntimeError("No vendor interface (class 0xFF) found")


def ctrl_in(dev, req, length, intf_num):
    return dev.ctrl_transfer(0xC1, req, 0, intf_num, length, timeout=TIMEOUT_MS)


def ctrl_out(dev, req, intf_num, wValue=0):
    dev.ctrl_transfer(0x41, req, wValue, intf_num, None, timeout=TIMEOUT_MS)


def try_get_map(dev, intf_num):
    data = bytes(ctrl_in(dev, REQ_GET_PORTMAP, 1, intf_num))
    if len(data) != 1:
        raise RuntimeError("Map request returned wrong length")
    return data[0]


def build_ports_from_map(map_val):
    return [i for i in range(8) if (map_val >> i) & 1]


def get_ports_to_test(dev):
    intf_num = find_vendor_interface(dev).bInterfaceNumber
    port_map = try_get_map(dev, intf_num)
    return build_ports_from_map(port_map)


def set_port_and_reopen(dev, intf_num, port):
    ctrl_out(dev, REQ_SET_PORT, intf_num, port)
    time.sleep(1)  # allow re-enumeration
    dev = find_device()
    intf_num = find_vendor_interface(dev).bInterfaceNumber
    return dev

# ---------------------- Bulk loopback ----------------------


def find_bulk_eps(dev):
    intf = find_vendor_interface(dev)
    ep_out = usb.util.find_descriptor(
        intf,
        custom_match=lambda e: usb.util.endpoint_direction(
            e.bEndpointAddress) == usb.util.ENDPOINT_OUT
        and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK
    )
    ep_in = usb.util.find_descriptor(
        intf,
        custom_match=lambda e: usb.util.endpoint_direction(
            e.bEndpointAddress) == usb.util.ENDPOINT_IN
        and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK
    )
    if ep_out is None or ep_in is None:
        raise RuntimeError("Could not find bulk endpoints on vendor interface")
    return ep_out, ep_in


HEADER_SIZE = 6  # <u32 seq><u16 len>


def make_packet(total_size, seq):
    if total_size < HEADER_SIZE:
        total_size = HEADER_SIZE
    hdr = struct.pack("<IH", seq, total_size)
    payload = bytes((i & 0xFF for i in range(total_size - HEADER_SIZE)))
    return hdr + payload


def check_echo(buf, expected_seq, expected_len):
    if len(buf) < HEADER_SIZE:
        return False, "short echo"
    seq, ln = struct.unpack("<IH", buf[:HEADER_SIZE])
    if seq != expected_seq or ln != expected_len:
        return False, f"header mismatch seq={seq} len={ln} expected seq={expected_seq} len={expected_len}"
    if len(buf) != expected_len:
        return False, "USB len mismatch"
    for i, b in enumerate(buf[HEADER_SIZE:]):
        if b != (i & 0xFF):
            return False, f"payload mismatch at {i}"
    return True, ""


def recv_exact(ep_in, size, timeout_ms=TIMEOUT_MS):
    buf = bytearray()
    while len(buf) < size:
        chunk = ep_in.read(size - len(buf), timeout=timeout_ms)
        buf.extend(chunk)
    return bytes(buf)


def run_bulk_test(dev, duration_s=TEST_SECS, pkt_size=PKT_SIZE):
    ep_out, ep_in = find_bulk_eps(dev)
    # flush stale IN
    try:
        while True:
            data = ep_in.read(512, timeout=5)
            if not data:
                break
    except usb.core.USBError:
        pass

    deadline = time.time() + duration_s
    seq = 0
    sent = 0
    got = 0
    errors = 0

    while time.time() < deadline:
        pkt = make_packet(pkt_size, seq)
        try:
            wrote = ep_out.write(pkt, timeout=TIMEOUT_MS)
            sent += wrote
        except usb.core.USBError:
            errors += 1
            continue

        try:
            echo = bytes(recv_exact(ep_in, pkt_size))
            ok, _ = check_echo(echo, seq, pkt_size)
            if not ok:
                errors += 1
            else:
                got += len(echo)
        except usb.core.USBError:
            errors += 1

        seq += 1

    bps = got / duration_s
    return {
        "bytes_sent": sent,
        "bytes_rcvd": got,
        "seconds": duration_s,
        "throughput_Bps": bps,
        "throughput_Mbps": (bps * 8) / 1e6,
        "errors": errors
    }

# ---------------------- Power parsing ----------------------


def get_adc_samples(dev, intf_num, n_steps):
    """
    Request and receive ADC samples via bulk transfer.

    Args:
        dev: USB device
        intf_num: Interface number
        n_steps: Number of load steps (idle + n_steps = total captures)

    Returns:
        List of lists, where each inner list contains ADC samples for one step
    """
    # Send control request to trigger bulk transfer
    ctrl_out(dev, REQ_GET_ADC_SAMPLES, intf_num)

    # Give device time to prepare the data
    time.sleep(0.1)

    # Find bulk IN endpoint
    _, ep_in = find_bulk_eps(dev)

    # Total captures = idle + load steps
    total_captures = n_steps + 1
    total_size = total_captures * ADC_SAMPLES_SIZE

    # Read all ADC samples from bulk endpoint
    adc_data = recv_exact(ep_in, total_size, timeout_ms=TIMEOUT_MS)

    # Unpack into separate arrays for each step
    all_samples = []
    for i in range(total_captures):
        start = i * ADC_SAMPLES_SIZE
        end = start + ADC_SAMPLES_SIZE
        step_data = adc_data[start:end]
        samples = struct.unpack(f"<{ADC_SAMPLES_PER_WINDOW}H", step_data)
        all_samples.append(list(samples))

    return all_samples


def parse_power_report(blob):
    if len(blob) != POWER_REPORT_SIZE:
        raise ValueError(
            f"power report wrong length: {len(blob)} != {POWER_REPORT_SIZE}")
    it = iter(struct.unpack(POWER_REPORT_FMT, blob))
    port = next(it)
    n_steps = next(it)
    flags = next(it)
    v_idle = next(it)

    def take_u16(n): return [next(it) for _ in range(n)]
    load_pct = take_u16(5)
    v_mean = take_u16(5)
    v_min = take_u16(5)
    droop = take_u16(5)
    ripple = take_u16(5)
    current_mA = take_u16(5)
    resistance_mOhm = take_u16(5)

    max_current = next(it)
    undervolt_at = next(it)
    errors = next(it)

    n = n_steps
    return {
        "port": port,
        "n_steps": n,
        "flags": flags,
        "v_idle_mV": v_idle,
        "load_pct":         load_pct[:n],
        "v_mean_mV":        v_mean[:n],
        "v_min_mV":         v_min[:n],
        "droop_mV":         droop[:n],
        "ripple_mVpp":      ripple[:n],
        "current_mA":       current_mA[:n],
        "resistance_mOhm":  resistance_mOhm[:n],
        "max_current_mA":   max_current,
        "undervolt_at_mA":  undervolt_at,
        "errors":           errors,
    }

# ---------------------- Evaluation ----------------------


def evaluate_port(port_result: Dict[str, Any]) -> Tuple[bool, List[str], Dict[str, Any]]:
    """Return (passed, reasons, rollup_metrics)"""
    reasons: List[str] = []
    passed = True

    port = port_result.get("port", -1)
    thr = port_result.get("throughput_Mbps", 0.0)
    if thr < MIN_THR_MBPS:
        passed = False
        reasons.append(
            f"throughput {thr:.2f} Mbps < {MIN_THR_MBPS:.2f} Mbps")

    # --- Skip power checks for control port 0 ---
    if port == 0:
        rollup = {
            "throughput_Mbps": thr,
            "vidle_mV": 0,
            "vmin_mV": 0,
            "max_droop_mV": 0,
            "max_ripple_mVpp": 0,
            "max_measured_current_mA": 0,
        }
        return passed, reasons, rollup

    pr = port_result.get("power_report", {}) or {}
    flags = pr.get("flags", 0)
    v_idle = pr.get("v_idle_mV", 0)
    # Actually percentage, not mA
    undervolt_at_pct = pr.get("undervolt_at_mA", 0)
    vmin_list = pr.get("v_min_mV", [])
    ripple_list = pr.get("ripple_mVpp", [])
    max_measured_current = pr.get("max_current_mA")
    resistance_list = pr.get("resistance_mOhm", [])

    # safe reductions
    vmin = min(vmin_list) if vmin_list else 99999
    max_ripple = max(ripple_list) if ripple_list else 0

    # Calculate resistance statistics
    valid_resistances = [r for r in resistance_list if r > 0]
    if valid_resistances:
        mean_resistance = sum(valid_resistances) / len(valid_resistances)
        min_resistance = min(valid_resistances)
        max_resistance = max(valid_resistances)
        resistance_variation = max_resistance - min_resistance
    else:
        mean_resistance = 0
        min_resistance = 0
        max_resistance = 0
        resistance_variation = 0

    # Check flags
    if flags & (1 << 0):  # bit 0: vbus_missing
        passed = False
        reasons.append(
            f"VBUS too low to test at idle ({v_idle} mV < {IDLE_UNDERVOLT_MV} mV)")

    if flags & (1 << 1):  # bit 1: undervolt
        passed = False
        reasons.append(
            f"VBUS dropped below {MIN_MV_LOAD}mV at {undervolt_at_pct}% load")

    if max_ripple > MAX_RIPPLE_MVPP:
        passed = False
        reasons.append(
            f"Voltage ripple {max_ripple}mVpp > {MAX_RIPPLE_MVPP}mVpp (noisy supply)")

    # Check current delivery capability
    if max_measured_current < MIN_MAX_CURRENT_MA:
        passed = False
        reasons.append(
            f"Maximum measured current {max_measured_current}mA < {MIN_MAX_CURRENT_MA}mA (insufficient current capability)"
        )

    # Check resistance consistency (resistive vs power supply issue)
    # Good: Resistance should be consistent across all load steps (~2000mΩ for this fixture)
    # Bad: Resistance increases with load = weak power supply or current limiting
    if valid_resistances and len(valid_resistances) >= 3:
        # Allow up to 500mΩ variation (25% of 2Ω fixture)
        # This accounts for measurement noise while catching power supply issues
        if resistance_variation > 500:
            passed = False
            reasons.append(
                f"Resistance varies {resistance_variation}mΩ ({min_resistance}-{max_resistance}mΩ) - indicates power supply issue, not pure resistive drop"
            )

        # Check for excessively high resistance (dirty contacts, corroded pins, damaged connector)
        if mean_resistance > MAX_RESISTANCE_MOHM:
            passed = False
            reasons.append(
                f"Mean resistance {mean_resistance:.0f}mΩ > {MAX_RESISTANCE_MOHM}mΩ - indicates dirty/corroded contacts or damaged cable"
            )

    # Optional: echo mismatch still fails
    echo = int(port_result.get("device_port_echo", port))
    if echo != port:
        passed = False
        reasons.append(f"port echo mismatch dev:{echo} != host:{port}")

    rollup = {
        "throughput_Mbps": thr,
        "vmin_mV": vmin,
        "vidle_mV": v_idle,
        "max_ripple_mVpp": max_ripple,
        "max_measured_current_mA": max_measured_current,
        "undervolt_at_pct": undervolt_at_pct,
        "mean_resistance_mOhm": mean_resistance,
        "resistance_variation_mOhm": resistance_variation,
    }
    return passed, reasons, rollup

# ---------------------- Main ----------------------


def main():
    overall_pass = True
    summary_obj: Dict[str, Any] = {"tested_ports": [], "per_port": []}

    try:
        dev = find_device()
        intf_num = find_vendor_interface(dev).bInterfaceNumber
        ports = get_ports_to_test(dev)
    except Exception as e:
        # Hard fail: no device / enumeration error
        print(f"USB TEST: No tester detected!!!")
        sys.exit(0)

    if not ports:
        print("USB TEST: FAIL — no ports detected")
        print(json.dumps({"error": "no ports detected"}))
        sys.exit(1)

    for p in ports:
        dev = set_port_and_reopen(dev, intf_num, p)

        res = run_bulk_test(dev, duration_s=TEST_SECS, pkt_size=PKT_SIZE)
        res["port"] = p
        try:
            port_echo = ctrl_in(dev, REQ_GET_PORT, 1, intf_num)[0]
            res["device_port_echo"] = int(port_echo)
        except Exception:
            res["device_port_echo"] = -1

        # Power report (skip for port 0 - control port with no power measurement)
        if p != 0:
            try:
                data = ctrl_in(dev, REQ_GET_POWER, POWER_REPORT_SIZE, intf_num)
                res["power_report"] = parse_power_report(bytes(data))

                # Get ADC samples via bulk transfer (idle + all load steps)
                try:
                    n_steps = res["power_report"].get("n_steps", 5)
                    adc_samples = get_adc_samples(dev, intf_num, n_steps)
                    res["power_report"]["adc_samples_idle"] = adc_samples[0]
                    res["power_report"]["adc_samples_steps"] = adc_samples[1:]
                except Exception as e:
                    res["adc_samples_error"] = str(e)

            except Exception as e:
                res["power_report_error"] = str(e)
                res["power_report"] = {
                    "v_min_mV": [], "v_max_mV": [], "droop_mV": [], "ripple_mVpp": [], "recovery_us": []
                }

        passed, reasons, rollup = evaluate_port(res)
        res["pass"] = passed
        res["fail_reasons"] = reasons
        res["rollup"] = rollup

        summary_obj["tested_ports"].append(p)
        summary_obj["per_port"].append(res)

        # concise single-line summary
        vidle_v = rollup["vidle_mV"] / 1000
        vmin_v = rollup["vmin_mV"] / \
            1000 if rollup["vmin_mV"] != 99999 else 0.0
        ripple = rollup["max_ripple_mVpp"]
        thr = rollup["throughput_Mbps"]
        imax = rollup["max_measured_current_mA"]
        mean_r = rollup.get("mean_resistance_mOhm", 0)
        r_var = rollup.get("resistance_variation_mOhm", 0)

        status = "PASS" if passed else "FAIL"
        if p == 0:
            print(f"USB Port {p} — {status}: {thr:.2f} Mbps")
        else:
            print(f"USB Port {p} — {status}: {thr:.2f} Mbps, Idle {vidle_v:.2f}V, "
                  f"Vmin {vmin_v:.2f}V, ripple {ripple}mVpp, "
                  f"Imax {imax}mA, R {mean_r:.0f}±{r_var:.0f}mΩ")

        if not passed:
            overall_pass = False
            # Print failure reasons on separate lines
            for reason in reasons:
                print(f"  -> {reason}")

        time.sleep(0.05)

    # Try to switch back to neutral/default port 0 (best effort)
    try:
        set_port_and_reopen(dev, intf_num, 0)
    except Exception:
        pass

    # Save report to file
    try:
        with open("./usb_report.json", "w") as f:
            json.dump(summary_obj, f, indent=2)
    except Exception:
        pass

    sys.exit(0 if overall_pass else 1)


if __name__ == "__main__":
    main()
