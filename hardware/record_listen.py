"""Record a Vector's state while the robot acts on its own: it is never given a command.

Authored-by: CPCS (cpcs-66ffc23), 2026-11-20. Runs on the wire-pod host with its anki_vector install,
fed on stdin; writes JSON lines to stdout. The rows are the ones hardware/record_barriered.py writes.

Why. The robot's own behaviours, started by a voice imperative ("Hey Vector, pop a wheelie"), only
run while no SDK client holds behaviour control. So this script connects WITHOUT asking for control
and only listens: state messages ("s" rows: gyro, accel, wheel speeds, head, lift, pose, pitch,
status, range, image time) and a battery reading ("b") at the start and the end.

Quiet mode. With wire-pod's quiet mode on, the pod holds behaviour control and the microphones are
muted, so a spoken imperative is not heard. If QUIET_LIFT=1 (the default), the script switches quiet
mode off through the pod at the start and puts it back as it found it at the end, also after an
error or a stop signal. The pod is POD_URL (default http://localhost:8080).

Settings: VECTOR_SERIAL (8 hex digits), VECTOR_IP (optional, a dotted IPv4 address), LISTEN_S
(10..300 s, default 180), QUIET_LIFT (0 or 1), NOTE (free text into the begin row).

It holds /tmp/vector-<serial>-takeover.lock while it runs, the same lock as record_barriered.py,
so the two never run on one robot at once. Exit codes: 0 recorded, 3 the lock is held.
"""
import datetime
import json
import os
import signal
import sys
import threading
import time
import urllib.request

import anki_vector
from anki_vector.events import Events

SERIAL = os.environ.get("VECTOR_SERIAL", "")
assert len(SERIAL) == 8 and all(c in "0123456789abcdef" for c in SERIAL.lower()), "VECTOR_SERIAL is 8 hex digits"
IP = os.environ.get("VECTOR_IP", "")
assert IP == "" or (IP.count(".") == 3 and all(p.isdigit() and 0 <= int(p) <= 255 for p in IP.split("."))), \
    "VECTOR_IP is a dotted IPv4 address"
LISTEN_S = float(os.environ.get("LISTEN_S", "180") or 0.0)
assert 10.0 <= LISTEN_S <= 300.0, "LISTEN_S must be 10..300 s"
QUIET_LIFT = os.environ.get("QUIET_LIFT", "1")
assert QUIET_LIFT in ("0", "1"), "QUIET_LIFT is 0 or 1"
POD_URL = os.environ.get("POD_URL", "http://localhost:8080")
assert POD_URL.startswith("http://") and " " not in POD_URL, "POD_URL is an http:// address"
LOCK = "/tmp/vector-%s-takeover.lock" % SERIAL
POD_TIMEOUT_S = 15.0
OUT_LOCK = threading.Lock()
T0 = time.monotonic()
STATE = {"stop": None}


def emit(kind, **fields):
    """One JSON line, safe from any thread."""
    row = {"k": kind, "t": round(time.monotonic() - T0, 4)}
    row.update(fields)
    with OUT_LOCK:
        sys.stdout.write(json.dumps(row) + "\n")
        sys.stdout.flush()


def on_state(robot, event_type, m):
    emit("s", gyro=[m.gyro.x, m.gyro.y, m.gyro.z], accel=[m.accel.x, m.accel.y, m.accel.z],
         wl=m.left_wheel_speed_mmps, wr=m.right_wheel_speed_mmps, head=m.head_angle_rad, lift=m.lift_height_mm,
         pose=[m.pose.x, m.pose.y, m.pose.z], yaw=m.pose_angle_rad, pitch=m.pose_pitch_rad, status=m.status,
         prox=m.prox_data.distance_mm, proxq=m.prox_data.signal_quality, img_ts=m.last_image_time_stamp)


def battery_read(robot):
    try:
        b = robot.get_battery_state()
        emit("b", volts=round(float(b.battery_volts), 4), level=int(b.battery_level), charging=bool(b.is_charging),
             on_platform=bool(b.is_on_charger_platform))
    except Exception as exc:  # a missed reading is recorded; the recording goes on
        emit("b", error=type(exc).__name__)


def quiet(method, enable=None):
    """GET the pod's quiet mode, or POST it on or off. The decoded answer, or None after recording the error."""
    url = "%s/api-sdk/quiet_mode?serial=%s" % (POD_URL, SERIAL)
    if enable is not None:
        url += "&enable=%s" % ("true" if enable else "false")
    try:
        req = urllib.request.Request(url, method=method)
        with urllib.request.urlopen(req, timeout=POD_TIMEOUT_S) as resp:
            body = json.loads(resp.read().decode("utf-8", "replace"))
    except Exception as exc:
        emit("c", name="quiet_failed", method=method, error=("%s: %s" % (type(exc).__name__, exc))[:160])
        return None
    fields = {k: body.get(k) for k in ("enabled", "active", "holding_reserve", "muted")}
    emit("c", name="quiet", method=method, **fields)
    return body


def on_signal(signum, frame):
    STATE["stop"] = signal.Signals(signum).name


def listen(robot):
    """Record for LISTEN_S, or until a stop signal."""
    emit("c", name="listening", seconds=LISTEN_S)
    end = time.monotonic() + LISTEN_S
    while time.monotonic() < end and not STATE["stop"]:
        time.sleep(0.2)
    emit("c", name="listened", stopped_by=STATE["stop"])


def main():
    try:
        fd = os.open(LOCK, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    except FileExistsError:
        emit("c", name="lock_held", path=LOCK)
        return 3
    os.write(fd, ("cpcs-66ffc23 listen-only recording pid %d\n" % os.getpid()).encode())
    os.close(fd)
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, on_signal)
    restore = None
    try:
        if QUIET_LIFT == "1":
            before = quiet("GET")
            if before is not None and before.get("enabled"):
                restore = True
                quiet("POST", enable=False)
        extra = {"ip": IP} if IP else {}
        robot = anki_vector.Robot(SERIAL, behavior_control_level=None, cache_animation_lists=False, **extra)
        emit("c", name="connect_to", serial=SERIAL, ip=IP or "from the SDK config", control="none")
        robot.connect(timeout=20)
        robot.events.subscribe(on_state, Events.robot_state)
        try:
            emit("c", name="begin", note=os.environ.get("NOTE", "")[:400],
                 utc=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds"))
            battery_read(robot)
            listen(robot)
            battery_read(robot)
        finally:
            robot.events.unsubscribe(on_state, Events.robot_state)
            robot.disconnect()
            emit("c", name="end", utc=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds"))
    finally:
        if restore:
            quiet("POST", enable=True)
        os.remove(LOCK)
        emit("c", name="lock_released")
    return 0


if __name__ == "__main__":
    sys.exit(main())
