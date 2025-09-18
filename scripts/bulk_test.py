import usb.core
import usb.util
import time
import sys
import os
import struct
import random


VID = 0x1209
PID = 0x4004

dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    sys.exit("Device not found")

# for cfg in dev:
#     for intf in cfg:
#         if dev.is_kernel_driver_active(intf.bInterfaceNumber):
    # print("KERNEL DRIVER IS ACTIVE?")
    # print(intf)
    # try:
    # dev.detach_kernel_driver(intf.bInterfaceNumber)
    # except Exception:
    #     pass

# dev.set_configuration()
cfg = dev.get_active_configuration()

intf = None
for i in cfg:
    if i.bInterfaceClass == 0xFF:
        intf = i
        break

if intf is None:
    sys.exit("No vendor interface (class 0xFF) found")

ep_out = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(
    e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
ep_in = usb.util.find_descriptor(intf, custom_match=lambda e: usb.util.endpoint_direction(
    e.bEndpointAddress) == usb.util.ENDPOINT_IN)
if ep_out is None or ep_in is None:
    sys.exit("Vendor endpoints not found")

# Test parameters
pkt = 64
secs = 3.0       # test duration
deadline = time.time() + secs
seq = 0
sent = 0
got = 0
errors = 0


def make_packet(n, seq):
    # simple header: [u32 seq][payload...pattern]
    hdr = struct.pack("<I", seq)
    body = bytes((i & 0xFF for i in range(n - 4)))
    return hdr + body


def check_packet(b):
    if (len(b) < pkt):
        return 0, False

    nonlocal_seq = struct.unpack("<I", b[:4])[0]
    # quick pattern check
    for idx, v in enumerate(b[4:]):
        if v != (idx & 0xFF):
            return nonlocal_seq, False
    return nonlocal_seq, True


# Prime and run

while time.time() < deadline:
    buf = make_packet(pkt, seq)
    ep_out.write(buf, timeout=1000)
    sent += 1 * pkt
    # read echo for last write
    echo = ep_in.read(pkt, timeout=2000).tobytes()

    # Small errors where it randomly sends 0 bytes
    if (len(echo) == 0):
        echo = ep_in.read(pkt, timeout=2000).tobytes()
    rseq, ok = check_packet(echo)
    if not ok or rseq != seq:
        print("sent")
        print(buf)
        print("received")
        print(echo)
        print("Error: {ok}, {rseq}")
        errors += 1
    got += len(echo)
    seq += 1


# drain if last echo
try:
    echo = ep_in.read(pkt, timeout=1000).tobytes()
    got += len(echo)
except usb.core.USBError:
    pass


bps = got / secs
print({
    "bytes_sent": sent,
    "bytes_rcvd": got,
    "seconds": secs,
    "throughput_Bps": bps,
    "throughput_Mbps": (bps * 8) / 1e6,
    "errors": errors
})
