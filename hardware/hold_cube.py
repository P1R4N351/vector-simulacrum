"""Hold a Vector's cube connected for hours and record what the cube reports. Never commands the robot.

Authored-by: CPCS (cpcs-66ffc23), 2026-11-20. Runs on the wire-pod host with its anki_vector install;
writes JSON lines to stdout.

Why. The robot connects its cube only while something subscribes to the connection. The SDK's
connect_cube subscribes "interactable" for 60 s (engine/components/cubes/appCubeConnectionSubscriber.cpp,
kCubeConnectionTimeout_s); 15 s after the last interactable subscriber goes the connection drops to a
background one, and 15 s later it disconnects (cubeConnectionCoordinator.cpp). Asking again renews the
60 s. So this script asks every RENEW_S seconds, and records:
  "e" rows  cube events: tapped, moved, stopped moving, up axis changed, connection state
  "r" rows  each renewal: whether it succeeded and which cube answered
  "b" rows  every BATTERY_S seconds: the robot's battery and the cube's (volts, level, factory id,
            seconds since the robot last heard the cube's battery)
It connects at RESERVE_CONTROL, the SDK's lowest control level: the SDK refuses connect_cube with no
control at all ("Unable to run 'connect_cube' because it requires behavior control", run R, 2026-11-21).
Reserve control holds the robot's idle behaviours off, as wire-pod's quiet mode does, so the robot stays
where it is; nothing is commanded. It does not stream the robot's state (16 messages a second would run
to tens of MB).

Settings: VECTOR_SERIAL (8 hex digits), VECTOR_IP (optional), HOLD_S (60..21600 s, default 10800),
RENEW_S (10..50 s, default 30; must stay under the 60 s expiry), BATTERY_S (30..600 s, default 60),
NOTE (free text into the begin row). At the end it lets the connection go (disconnect_cube). It also ends
early if the robot's battery reads below LOW_V (3.5 V, chosen: under the SDK's "At 3.6V, the robot is
approaching low charge"), so that reserve control is never held over a robot running down.

It holds /tmp/vector-<serial>-takeover.lock, the same lock as the other recorders. Exit codes: 0 done,
3 the lock is held. SIGTERM, SIGINT and SIGHUP end it early and cleanly.
"""
import datetime
import json
import os
import signal
import sys
import threading
import time

import anki_vector
from anki_vector.connection import ControlPriorityLevel
from anki_vector.events import Events

SERIAL = os.environ.get("VECTOR_SERIAL", "")
assert len(SERIAL) == 8 and all(c in "0123456789abcdef" for c in SERIAL.lower()), "VECTOR_SERIAL is 8 hex digits"
IP = os.environ.get("VECTOR_IP", "")
assert IP == "" or (IP.count(".") == 3 and all(p.isdigit() and 0 <= int(p) <= 255 for p in IP.split("."))), \
    "VECTOR_IP is a dotted IPv4 address"
HOLD_S = float(os.environ.get("HOLD_S", "10800") or 0.0)
assert 60.0 <= HOLD_S <= 21600.0, "HOLD_S must be 60..21600 s"
RENEW_S = float(os.environ.get("RENEW_S", "30") or 0.0)
assert 10.0 <= RENEW_S <= 50.0, "RENEW_S must be 10..50 s, under the SDK subscription's 60 s"
BATTERY_S = float(os.environ.get("BATTERY_S", "60") or 0.0)
assert 30.0 <= BATTERY_S <= 600.0, "BATTERY_S must be 30..600 s"
LOCK = "/tmp/vector-%s-takeover.lock" % SERIAL
TICK_S = 0.5
LOW_V = 3.5
CUBE_EVENTS = ("object_tapped", "object_moved", "object_stopped_moving", "object_up_axis_changed",
               "object_connection_state")
OUT_LOCK = threading.Lock()
T0 = time.monotonic()
STATE = {"stop": None, "volts": None}


def emit(kind, **fields):
    """One JSON line, safe from any thread."""
    row = {"k": kind, "t": round(time.monotonic() - T0, 4)}
    row.update(fields)
    with OUT_LOCK:
        sys.stdout.write(json.dumps(row) + "\n")
        sys.stdout.flush()


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds")


def on_cube_event(robot, event_type, event):
    """Every cube event, with the fields that identify it; anything else in the message is left out."""
    fields = {}
    for name in ("object_id", "timestamp", "up_axis", "connected", "factory_id", "object_type"):
        value = getattr(event, name, None)
        if value is not None:
            fields[name] = value if isinstance(value, (bool, int, float)) else str(value)[:60]
    emit("e", event=str(event_type), utc=utc(), **fields)


def renew(robot):
    """Ask for the cube connection again; the robot's answer, or the error."""
    try:
        reply = robot.world.connect_cube()
        emit("r", success=bool(getattr(reply, "success", False)), factory_id=str(getattr(reply, "factory_id", ""))[:40])
    except Exception as exc:  # recorded; the next renewal tries again
        emit("r", error=("%s: %s" % (type(exc).__name__, exc))[:160])


def battery_read(robot):
    try:
        b = robot.get_battery_state()
        cube = getattr(b, "cube_battery", None)
        STATE["volts"] = float(b.battery_volts)
        fields = {"volts": round(float(b.battery_volts), 4), "charging": bool(b.is_charging),
                  "on_platform": bool(b.is_on_charger_platform)}
        if cube is not None and getattr(cube, "factory_id", ""):
            fields.update(cube_volts=round(float(cube.battery_volts), 4), cube_level=int(cube.level),
                          cube_id=str(cube.factory_id)[:40], cube_age_s=round(float(cube.time_since_last_reading_sec), 1))
        emit("b", **fields)
    except Exception as exc:  # a missed reading is recorded; the hold goes on
        emit("b", error=type(exc).__name__)


def on_signal(signum, frame):
    STATE["stop"] = signal.Signals(signum).name


def hold(robot):
    """Renew every RENEW_S and read the batteries every BATTERY_S, for HOLD_S or until a stop signal."""
    start = time.monotonic()
    next_renew = next_battery = start
    emit("c", name="holding", seconds=HOLD_S, renew_s=RENEW_S, battery_s=BATTERY_S, utc=utc())
    for _ in range(int(HOLD_S / TICK_S) + 1):
        now = time.monotonic()
        if STATE["stop"] or now - start >= HOLD_S:
            break
        if now >= next_renew:
            renew(robot)
            next_renew = now + RENEW_S
        if now >= next_battery:
            battery_read(robot)
            next_battery = now + BATTERY_S
            if STATE["volts"] is not None and STATE["volts"] < LOW_V:
                STATE["stop"] = "low_battery"
                break
        time.sleep(TICK_S)
    emit("c", name="held", seconds=round(time.monotonic() - start, 1), stopped_by=STATE["stop"], utc=utc())


def main():
    try:
        fd = os.open(LOCK, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    except FileExistsError:
        emit("c", name="lock_held", path=LOCK)
        return 3
    os.write(fd, ("cpcs-66ffc23 cube hold pid %d\n" % os.getpid()).encode())
    os.close(fd)
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, on_signal)
    try:
        extra = {"ip": IP} if IP else {}
        robot = anki_vector.Robot(SERIAL, behavior_control_level=ControlPriorityLevel.RESERVE_CONTROL,
                                  cache_animation_lists=False, **extra)
        emit("c", name="connect_to", serial=SERIAL, ip=IP or "from the SDK config", control="reserve")
        robot.connect(timeout=20)
        handlers = [(getattr(Events, name), name) for name in CUBE_EVENTS]
        for event, _ in handlers:
            robot.events.subscribe(on_cube_event, event)
        try:
            emit("c", name="begin", note=os.environ.get("NOTE", "")[:400], utc=utc())
            hold(robot)
            battery_read(robot)
        finally:
            for event, _ in handlers:
                robot.events.unsubscribe(on_cube_event, event)
            try:
                robot.world.disconnect_cube()
                emit("c", name="cube_released")
            except Exception as exc:  # the robot drops it by itself 30 s after the last renewal
                emit("c", name="cube_release_failed", error=type(exc).__name__)
            robot.disconnect()
            emit("c", name="end", utc=utc())
    finally:
        os.remove(LOCK)
        emit("c", name="lock_released")
    return 0


if __name__ == "__main__":
    sys.exit(main())
