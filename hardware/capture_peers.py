"""Camera frames for teaching Vectors to see each other: one robot holds still and streams what it sees.

Authored-by: CPCS (cpcs-66ffc23), 2026-11-21. Runs on the wire-pod host with its anki_vector install,
fed on stdin; writes JSON lines to stdout, frames as base64 JPEG, and leaves nothing behind but a lock
file it removes. hardware/save_capture.py turns the stream into image files and a manifest.

Why. It was decided on 2026-11-21 that images of
the household's own Vectors, with and without a Vector in view, come before any detector is chosen.
Banerjee's public set has no images without a Vector, and its test photos are neighbours of its training
photos, so it cannot say how a detector does in a new scene.

What it does, in order:
1. Takes /tmp/vector-<serial>-takeover.lock, or exits 3 without connecting.
   Then asks the pod for Cryosleep's mute (/api-sdk/cryo_mute) and exits 6 without connecting unless it
   reads off: Cryosleep is the household's overnight sleep, and it withholds the camera on purpose.
2. Connects with DEFAULT_PRIORITY behaviour control and waits up to CONTROL_WAIT_S for the grant, or
   exits 5. Held control keeps the robot's own behaviours off, so it does not drive off its charger.
   It commands nothing: no wheels, head, lift, sound or screen.
3. If QUIET_LIFT=1 and the pod's quiet mode is on, lifts it, because quiet mode on gives no camera
   frames. It lifts it only once control is held, and puts it back before control is let go.
4. Starts the camera feed and, every 1/FPS s, emits the newest frame if its image id is new.
5. Stops after CAPTURE_S, on a stop signal, if control is lost, or if no new frame came for STALL_S.

Settings (environment):
  VECTOR_SERIAL  the observing robot, 8 hex digits        VECTOR_IP   its address, optional
  IN_VIEW        what the operator put in front of it: "none", or units from unit 1,unit 2,unit 3 joined
                 by commas. It is the scene's plan; per-frame boxes come later, from labelling.
  SCENE          free text: surface, light, background (at most 400 characters)
  CAPTURE_S      10..600, default 120      FPS     0.2..5, default 1      STALL_S  2..30, default 10
  JPEG_QUALITY   50..95, default 90        QUIET_LIFT  0 or 1, default 1  POD_URL  default http://localhost:8080
Exit codes: 0 done, 3 lock held, 5 control not granted, 6 Cryosleep on or not known to be off.
"""

import base64
import hashlib
import io
import json
import os
import signal
import sys
import threading
import time
import urllib.error
import urllib.request

import anki_vector
from anki_vector.connection import ControlPriorityLevel

UNITS = ("Unit 1", "Unit 2", "Unit 3")
SERIAL = os.environ.get("VECTOR_SERIAL", "")
assert len(SERIAL) == 8 and all(c in "0123456789abcdef" for c in SERIAL.lower()), "VECTOR_SERIAL is 8 hex digits"
IP = os.environ.get("VECTOR_IP", "")
assert IP == "" or (IP.count(".") == 3 and all(p.isdigit() and 0 <= int(p) <= 255 for p in IP.split("."))), \
    "VECTOR_IP is a dotted IPv4 address"
IN_VIEW = tuple(p.strip() for p in os.environ.get("IN_VIEW", "").split(",") if p.strip())
assert IN_VIEW == ("none",) or (IN_VIEW and len(set(IN_VIEW)) == len(IN_VIEW) and all(u in UNITS for u in IN_VIEW)), \
    "IN_VIEW is none, or units from %s" % ",".join(UNITS)
SCENE = os.environ.get("SCENE", "")
assert len(SCENE) <= 400, "SCENE is at most 400 characters"
CAPTURE_S = float(os.environ.get("CAPTURE_S", "120"))
assert 10.0 <= CAPTURE_S <= 600.0, "CAPTURE_S is 10..600"
FPS = float(os.environ.get("FPS", "1"))
assert 0.2 <= FPS <= 5.0, "FPS is 0.2..5"
STALL_S = float(os.environ.get("STALL_S", "10"))
assert 2.0 <= STALL_S <= 30.0, "STALL_S is 2..30"
JPEG_QUALITY = int(os.environ.get("JPEG_QUALITY", "90"))
assert 50 <= JPEG_QUALITY <= 95, "JPEG_QUALITY is 50..95"
QUIET_LIFT = os.environ.get("QUIET_LIFT", "1")
assert QUIET_LIFT in ("0", "1"), "QUIET_LIFT is 0 or 1"
POD_URL = os.environ.get("POD_URL", "http://localhost:8080")
assert POD_URL.startswith("http://") and " " not in POD_URL, "POD_URL is an http:// address"
POD_TIMEOUT_S = 15.0
CONTROL_WAIT_S = 10.0
POLL_S = 0.05
MAX_TICKS = int(CAPTURE_S * FPS)
assert 1 <= MAX_TICKS <= 3000, "at most 3000 frames"
LOCK = "/tmp/vector-%s-takeover.lock" % SERIAL
STOP_SIGNALS = (signal.SIGTERM, signal.SIGINT, signal.SIGHUP)
OUT_LOCK = threading.Lock()
T0 = time.monotonic()
STATE = {"stop": None}


def emit(kind, **fields):
    """One JSON line, safe from any thread."""
    row = {"k": kind, "t": round(time.monotonic() - T0, 4)}
    row.update(fields)
    with OUT_LOCK:
        sys.stdout.write(json.dumps(row, separators=(",", ":")) + "\n")
        sys.stdout.flush()


def on_signal(signum, frame):
    STATE["stop"] = signal.Signals(signum).name


def quiet(method, enable=None):
    """GET the pod's quiet mode, or POST it on or off. The decoded answer, or None after recording the error."""
    url = "%s/api-sdk/quiet_mode?serial=%s" % (POD_URL, SERIAL)
    if enable is not None:
        url += "&enable=%s" % ("true" if enable else "false")
    try:
        req = urllib.request.Request(url, method=method)
        with urllib.request.urlopen(req, timeout=POD_TIMEOUT_S) as resp:
            body = json.loads(resp.read().decode("utf-8", "replace"))
    except Exception as exc:  # a pod error is recorded; the caller decides what it means
        emit("c", name="quiet_failed", method=method, error=("%s: %s" % (type(exc).__name__, exc))[:160])
        return None
    emit("c", name="quiet", method=method, **{k: body.get(k) for k in ("enabled", "active", "holding_reserve", "muted")})
    return body


def cryosleep_clear():
    """True only if the pod says Cryosleep's mute is off, or the pod has no cryo_mute endpoint (HTTP 404),
    which means Cryosleep cannot run there. Any other error is False: the script cannot tell, so it waits."""
    url = "%s/api-sdk/cryo_mute?serial=%s" % (POD_URL, SERIAL)
    try:
        with urllib.request.urlopen(urllib.request.Request(url, method="GET"), timeout=POD_TIMEOUT_S) as resp:
            body = json.loads(resp.read().decode("utf-8", "replace"))
    except urllib.error.HTTPError as exc:
        emit("c", name="cryo_check", status=exc.code)
        exc.close()
        return exc.code == 404
    except Exception as exc:  # unreachable pod or bad answer: not clear
        emit("c", name="cryo_check", error=("%s: %s" % (type(exc).__name__, exc))[:160])
        return False
    muted = body.get("muted") if isinstance(body, dict) else None
    emit("c", name="cryo_check", muted=muted)
    return muted is False


def has_control(robot):
    """Control as the connection's control_granted_event says (the has_control flag is not cleared on loss)."""
    granted = getattr(robot.conn, "control_granted_event", None)
    if granted is not None:
        return bool(granted.is_set())
    events = getattr(robot.conn, "_control_events", None)
    return bool(getattr(events, "has_control", False))


def wait_for_control(robot):
    """True once control is granted, False after CONTROL_WAIT_S."""
    for _ in range(int(CONTROL_WAIT_S / POLL_S) + 1):
        if has_control(robot):
            return True
        time.sleep(POLL_S)
    return False


def battery_row(robot, when):
    try:
        b = robot.get_battery_state()
        emit("b", when=when, volts=round(float(b.battery_volts), 4), level=int(b.battery_level),
             charging=bool(b.is_charging), on_platform=bool(b.is_on_charger_platform))
    except Exception as exc:  # a missed reading is recorded; the capture goes on
        emit("b", when=when, error=type(exc).__name__)


def frame_row(robot, image):
    """Emit one frame with the robot's own head, lift and charger state at the time it was read."""
    buf = io.BytesIO()
    image.raw_image.save(buf, format="JPEG", quality=JPEG_QUALITY)
    data = buf.getvalue()
    assert data[:2] == b"\xff\xd8", "the encoder did not produce a JPEG"
    width, height = image.raw_image.size
    emit("f", id=int(image.image_id), w=int(width), h=int(height), head=round(float(robot.head_angle_rad), 4),
         lift=round(float(robot.lift_height_mm), 2), on_charger=bool(robot.status.is_on_charger),
         sha256=hashlib.sha256(data).hexdigest(), jpeg=base64.b64encode(data).decode("ascii"))


def wait_tick(deadline):
    """Sleep in short steps until deadline or a stop signal; bounded by the tick length."""
    for _ in range(int(1.0 / (FPS * POLL_S)) + 2):
        if STATE["stop"] or time.monotonic() >= deadline:
            return
        time.sleep(min(POLL_S, max(0.0, deadline - time.monotonic())))


def capture(robot):
    """The capture loop. Returns why it stopped."""
    last_id, last_new, frames = None, time.monotonic(), 0
    start = time.monotonic()
    for tick in range(MAX_TICKS):
        wait_tick(start + (tick + 1) / FPS)
        if STATE["stop"]:
            return "signal:" + STATE["stop"], frames
        if not has_control(robot):
            return "control_lost", frames
        image = robot.camera.latest_image
        image_id = getattr(image, "image_id", None)
        if image is not None and image_id is not None and image_id != last_id:
            frame_row(robot, image)
            last_id, last_new, frames = image_id, time.monotonic(), frames + 1
        elif time.monotonic() - last_new >= STALL_S:
            return "stall", frames
    return "done", frames


def observe(robot):
    """Lift quiet mode if asked, capture, and always put quiet mode back. Control is already held."""
    restore = False
    if QUIET_LIFT == "1":
        before = quiet("GET")
        if before is not None and before.get("enabled"):
            restore = True
            quiet("POST", enable=False)
    try:
        robot.camera.init_camera_feed()
        try:
            battery_row(robot, "begin")
            emit("c", name="begin", in_view=list(IN_VIEW), scene=SCENE, capture_s=CAPTURE_S, fps=FPS,
                 stall_s=STALL_S, jpeg_quality=JPEG_QUALITY, serial=SERIAL)
            why, frames = capture(robot)
            emit("c", name="end", why=why, frames=frames)
            battery_row(robot, "end")
        finally:
            robot.camera.close_camera_feed()
    finally:
        if restore:
            quiet("POST", enable=True)


def main():
    try:
        fd = os.open(LOCK, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    except FileExistsError:
        emit("c", name="lock_held", path=LOCK)
        return 3
    os.write(fd, ("cpcs-66ffc23 peer capture pid %d\n" % os.getpid()).encode())
    os.close(fd)
    for sig in STOP_SIGNALS:
        signal.signal(sig, on_signal)
    try:
        if not cryosleep_clear():
            emit("c", name="cryosleep_not_clear")
            return 6
        extra = {"ip": IP} if IP else {}
        robot = anki_vector.Robot(SERIAL, behavior_control_level=ControlPriorityLevel.DEFAULT_PRIORITY,
                                  cache_animation_lists=False, **extra)
        emit("c", name="connect_to", serial=SERIAL, ip=IP or "from the SDK config", control="default")
        robot.connect(timeout=20)
        try:
            if not wait_for_control(robot):
                emit("c", name="control_not_granted", waited_s=CONTROL_WAIT_S)
                return 5
            observe(robot)
        finally:
            robot.disconnect()
            emit("c", name="disconnected")
    finally:
        os.remove(LOCK)
    return 0


if __name__ == "__main__":
    sys.exit(main())
