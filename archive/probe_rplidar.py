#!/usr/bin/env python3
"""Probe an RPLIDAR over raw serial — no pyserial, no ROS, no driver install.

    python3 real_world/bringup/probe_rplidar.py [/dev/ttyUSB0] [baud]

Answers the two questions gate H2 opens with, before any ROS package exists:
  * is the device actually an RPLIDAR, and which model / firmware?
  * is the baud right?  A1M8 is 115200; the A3/S2 are 256000, and the wrong baud gives a SILENT
    no-data start rather than an error — which is why this is worth checking by hand once.

Sends only STOP / GET_HEALTH / GET_INFO. It never starts a scan, so the motor stays idle.

WHY termios AND NOT stty
------------------------
An earlier version shelled out to `stty` and failed about one run in four. The failure was
all-or-nothing per process: a good run answered in 0.6 s, a bad run read ZERO bytes across four
retries and 10 s. In-session it was 20/20, so the head was fine — the port was simply misconfigured
for the whole life of the bad process, because `stty` runs asynchronously against the driver's
re-initialisation of the line on open(). Setting the attributes on the fd we already hold is
atomic — but it only lifted the success rate to ~73%, so that was NOT the whole story.

Measured, so the next person does not repeat the hunt:
  * within a single open session: 20/20 GET_HEALTH, zero failures.
  * across process restarts: ~70-90%, and a bad process reads ZERO bytes for its whole life.
  * forcing DTR/RTS high or low, and settles up to 1 s, made no difference beyond noise (n=12).
Root cause NOT established; it lives somewhere in the CP2102 reopen path. It is a property of
reopening, not of the sensor — which reports health GOOD and a stable serial every time it answers.
The ROS driver holds the port open continuously, i.e. the 20/20 regime, so this is a quirk of
hand-probing only. Rather than keep guessing, the probe simply reopens on failure.
"""
import os
import sys
import termios
import time

SYNC = b"\xA5"
STOP, GET_HEALTH, GET_INFO = b"\x25", b"\x52", b"\x50"
HEALTH = {0: "GOOD", 1: "WARNING", 2: "ERROR"}
BAUDS = {"115200": termios.B115200, "256000": getattr(termios, "B256000", None),
         "460800": termios.B460800, "921600": termios.B921600,
         "1000000": termios.B1000000}


def configure(fd, baud):
    """8N1 raw, no flow control, non-blocking reads. Applied to the fd we already hold."""
    speed = BAUDS.get(baud)
    if speed is None:
        raise SystemExit(f"baud {baud} not supported by termios on this platform")
    iflag, oflag, cflag, lflag, _, _, cc = termios.tcgetattr(fd)
    cflag = termios.CLOCAL | termios.CREAD | termios.CS8
    iflag = oflag = lflag = 0
    cc = list(cc)
    cc[termios.VMIN], cc[termios.VTIME] = 0, 1
    termios.tcsetattr(fd, termios.TCSANOW,
                      [iflag, oflag, cflag, lflag, speed, speed, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)


def drain(fd):
    while True:
        try:
            if not os.read(fd, 4096):
                return
        except BlockingIOError:
            return


def request(fd, cmd, want, timeout=2.0, attempts=3):
    """Send a command, consume the 7-byte response descriptor, return `want` payload bytes."""
    last = None
    for n in range(attempts):
        try:
            return _once(fd, cmd, want, timeout)
        except (TimeoutError, ValueError) as e:
            last = e
            drain(fd)
            time.sleep(0.2 * (n + 1))
    raise last


def _once(fd, cmd, want, timeout):
    os.write(fd, SYNC + cmd)
    buf, deadline = b"", time.time() + timeout
    while len(buf) < 7 + want and time.time() < deadline:
        try:
            chunk = os.read(fd, 7 + want - len(buf))
        except BlockingIOError:
            chunk = b""
        if chunk:
            buf += chunk
        else:
            time.sleep(0.005)
    if len(buf) < 7 + want:
        raise TimeoutError(f"got {len(buf)}/{7 + want} bytes — wrong baud, wrong port, or asleep")
    if buf[0:2] != b"\xA5\x5A":
        raise ValueError(f"bad response descriptor {buf[:7].hex()} — not an RPLIDAR at this baud")
    return buf[7:]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = sys.argv[2] if len(sys.argv) > 2 else "115200"

    # Reopen on failure: a bad open stays bad for the life of the fd (see module docstring), so
    # retrying inside the session cannot help — only a fresh open can.
    for attempt in range(4):
        try:
            probe(port, baud)
            return
        except (TimeoutError, ValueError) as e:
            if attempt == 3:
                raise SystemExit(f"probe failed after 4 opens: {e}")
            print(f"  (open {attempt + 1} unresponsive, reopening)", file=sys.stderr)
            time.sleep(0.5)


def probe(port, baud):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure(fd, baud)
        time.sleep(0.3)                    # DTR toggles on open; let the head settle
        os.write(fd, SYNC + STOP)          # make sure it is not mid-scan
        time.sleep(0.05)
        drain(fd)

        h = request(fd, GET_HEALTH, 3)
        print(f"health   : {HEALTH.get(h[0], h[0])}  error_code={int.from_bytes(h[1:3], 'little')}")

        i = request(fd, GET_INFO, 20)
        print(f"model    : 0x{i[0]:02X}")
        print(f"firmware : {i[2]}.{i[1]:02d}")
        print(f"hardware : {i[3]}")
        print(f"serial   : {i[4:20].hex().upper()}")
        print(f"port     : {port} @ {baud} baud  -> OK")
    finally:
        try:
            os.write(fd, SYNC + STOP)
        except OSError:
            pass
        os.close(fd)


if __name__ == "__main__":
    main()
