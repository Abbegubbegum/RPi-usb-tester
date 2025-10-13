
import usb.core
import usb.util
import time
import sys
import struct
import os
import json

# ---------------------- Config ----------------------
VID = 0x1209
PID = 0x4004

# Request IDs (match your firmware)
REQ_GET_PORT = 0x01  # IN:  u8 port
REQ_SET_PORT = 0x02  # OUT: wValue = port
REQ_GET_POWER = 0x03  # IN:  TBD (not used here)
REQ_GET_PORTMAP = 0x10  # IN:  u8 port_map (bitmask of available ports)

TEST_SECS = 3.0
PKT_SIZE = 1024        # total packet size including header
TIMEOUT_MS = 1000

# ---------------------- USB helpers ----------------------


def find_device():
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        raise RuntimeError(
            "Device not found (VID=0x%04X, PID=0x%04X)" % (VID, PID))
    # Select a configuration but don't auto-claim interfaces; PyUSB does lazy claims
    try:
        dev.set_configuration()
    except usb.core.USBError:
        pass
    return dev


def find_vendor_interface(dev):
    for cfg in dev:
        for intf in cfg:
            if intf.bInterfaceClass == 0xFF:  # vendor
                return intf
    raise RuntimeError("No vendor interface (class 0xFF) found")


def ctrl_in(dev, req, length, intf_num):
    # Try interface recipient (0xC1) first if intf_num is provided, otherwise device recipient (0xC0)

    return dev.ctrl_transfer(0xC1, req, 0, intf_num, length, timeout=TIMEOUT_MS)


def ctrl_out(dev, req, intf_num, wValue=0):

    dev.ctrl_transfer(0x41, req, wValue, intf_num,
                      None, timeout=TIMEOUT_MS)

# ---------------------- Maps & port selection ----------------------


def try_get_map(dev, intf_num):
    data = bytes(ctrl_in(dev, REQ_GET_PORTMAP, 1, intf_num))
    if len(data) != 1:
        raise RuntimeError("Map request returned wrong length")

    return data[0]


def build_ports_from_map(map_val):
    # size of map_val is 1 byte, so max 8 ports
    ports = [i for i in range(8) if (map_val >> i) & 1]
    return ports


def get_ports_to_test(dev):
    intf_num = find_vendor_interface(dev).bInterfaceNumber

    # First prefer ENUM map (actual data path works)

    port_map = try_get_map(dev, intf_num)
    ports = build_ports_from_map(port_map)
    return ports


def set_port_and_reopen(dev, intf_num, port):
    ctrl_out(dev, REQ_SET_PORT, intf_num, port)
    # After a port switch the device may have re-enumerated, so refind it
    time.sleep(1)  # let device settle
    dev = find_device()
    intf_num = find_vendor_interface(dev).bInterfaceNumber
    return dev

# ---------------------- Bulk loopback test ----------------------


def find_bulk_eps(dev):
    intf = find_vendor_interface(dev)
    # Expect one BULK OUT and one BULK IN
    ep_out = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(
        e.bEndpointAddress) == usb.util.ENDPOINT_OUT and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK)
    ep_in = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(
        e.bEndpointAddress) == usb.util.ENDPOINT_IN and usb.util.endpoint_type(e.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK)
    if ep_out is None or ep_in is None:
        raise RuntimeError("Could not find bulk endpoints on vendor interface")
    return ep_out, ep_in


HEADER_SIZE = 6  # <u32 seq><u16 len>


def make_packet(total_size, seq):
    if total_size < HEADER_SIZE:
        total_size = HEADER_SIZE
    hdr = struct.pack("<IH", seq, total_size)
    # Payload pattern: incremental bytes 0..255 repeating
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
    # Verify payload pattern
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

    # Prime: flush any stale IN data
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
            # if len(echo) == 0:
            #     echo = bytes(ep_in.read(pkt_size, timeout=TIMEOUT_MS))
            ok, why = check_echo(echo, seq, pkt_size)
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


POWER_REPORT_FMT = "<BBB" + "HH" + "5H"*8 + "HHH"
POWER_REPORT_SIZE = struct.calcsize(POWER_REPORT_FMT)


def parse_power_report(blob):
    if len(blob) != POWER_REPORT_SIZE:
        raise ValueError(
            f"power report wrong length: {len(blob)} != {POWER_REPORT_SIZE}")
    it = iter(struct.unpack(POWER_REPORT_FMT, blob))
    port = next(it)
    n_steps = next(it)
    flags = next(it)
    maxpower = next(it)
    v_idle = next(it)

    # helper to pull N uint16s
    def take_u16(n): return [next(it) for _ in range(n)]
    loads = take_u16(5)
    v_mean = take_u16(5)
    v_min = take_u16(5)
    v_max = take_u16(5)
    droop = take_u16(5)
    ripple = take_u16(5)
    current_mA = take_u16(5)
    recovery_us = take_u16(5)

    max_current = next(it)
    ocp_at = next(it)
    errors = next(it)
    return {
        "port": port,
        "n_steps": n_steps,
        "flags": flags,
        "maxpower_mA": maxpower,
        "v_idle_mV": v_idle,
        "loads_mA":        loads[:n_steps],
        "v_mean_mV":       v_mean[:n_steps],
        "v_min_mV":        v_min[:n_steps],
        "v_max_mV":        v_max[:n_steps],
        "droop_mV":        droop[:n_steps],
        "ripple_mVpp":     ripple[:n_steps],
        "current_mA":      current_mA[:n_steps],
        "recovery_us":     recovery_us[:n_steps],
        "max_current_mA":  max_current,
        "ocp_at_mA":       ocp_at,
        "errors":          errors,
    }


# ---------------------- Main ----------------------


def main():
    dev = find_device()
    intf_num = find_vendor_interface(dev).bInterfaceNumber

    ports = get_ports_to_test(dev)
    if not ports:
        print({"error": "no ports detected"})
        return

    results = []
    for p in ports:
        # Switch to port p
        dev = set_port_and_reopen(dev, intf_num, p)
        # Run bulk loopback on this path
        res = run_bulk_test(dev, duration_s=TEST_SECS, pkt_size=PKT_SIZE)
        res["port"] = p
        # Query device for reported active port
        port_echo = ctrl_in(dev, REQ_GET_PORT, 1, intf_num)[0]
        res["device_port_echo"] = int(port_echo)
        # Query device for power test
        data = ctrl_in(dev, REQ_GET_POWER, POWER_REPORT_SIZE, intf_num)
        res["power_report"] = parse_power_report(bytes(data))

        results.append(res)
        # Small pause between ports
        time.sleep(0.05)

    # Print a compact summary
    summary = {
        "tested_ports": ports,
        "per_port": results
    }
    print(summary)

    with open("report.json", "w") as f:
        json.dump(summary, f, indent=2)


if __name__ == "__main__":
    main()
