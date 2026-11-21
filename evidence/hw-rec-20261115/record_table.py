"""Record Vector's responses to a short scripted sequence, for comparison with the simulacrum.
fed on stdin; writes JSON lines to stdout and leaves nothing behind but a lock file it removes.

Safety, for a robot on an open-edged table:
- Control is requested at DEFAULT priority, so the robot's own cliff reaction stays on
  and can take control away. The sequence stops moving the moment it does.
- Every motion is bounded in time and followed by stop_all_motors.
- Driving is the SDK's own drive_off_charger, then spins in place and 25 mm steps.
- Lowest speeds first. A cliff, pick-up or fall flag, or loss of control, ends all motion.
"""
import json
import os
import signal
import sys
import threading
import time

import anki_vector
from anki_vector.connection import ControlPriorityLevel
from anki_vector.events import Events
from anki_vector.util import degrees

SERIAL = "00000000"  # set to your robot's serial number
LOCK = "/tmp/vector-%s-takeover.lock" % SERIAL
POLL_S = 0.02
OUT_LOCK = threading.Lock()
T0 = time.monotonic()
STATE = {"robot": None, "abort": None}


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
         prox=m.prox_data.distance_mm, proxq=m.prox_data.signal_quality)


def has_control(robot):
    events = getattr(robot.conn, "_control_events", None)
    return bool(getattr(events, "has_control", False))


def hazard(robot):
    """A reason to stop all motion, or None."""
    st = robot.status
    if st.is_cliff_detected:
        return "cliff_detected"
    if st.is_picked_up or st.is_being_held:
        return "picked_up"
    if st.is_falling:
        return "falling"
    if not has_control(robot):
        return "control_lost"
    return None


def stop(robot, why):
    try:
        robot.motors.stop_all_motors()
    except Exception as exc:  # the stop is best effort; the reason is still recorded
        emit("c", name="stop_failed", why=why, error=type(exc).__name__)
        return
    emit("c", name="stop", why=why)


def wheels_for(robot, left, right, seconds, label):
    """Hold two wheel speeds for a bounded time, watching for hazards. Returns False if motion must end."""
    assert 0.0 < seconds <= 3.0 and abs(left) <= 100.0 and abs(right) <= 100.0
    if STATE["abort"]:
        return False
    reason = hazard(robot)
    if reason:
        STATE["abort"] = reason
        emit("c", name="skip", label=label, why=reason)
        return False
    emit("c", name="wheels", label=label, left=left, right=right, seconds=seconds)
    robot.motors.set_wheel_motors(left, right)
    emit("c", name="wheels_sent", label=label)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        reason = hazard(robot)
        if reason:
            STATE["abort"] = reason
            stop(robot, reason)
            return False
        time.sleep(POLL_S)
    stop(robot, "segment_end")
    time.sleep(1.0)
    return True


def step(robot, name, call, **args):
    """One blocking SDK behaviour call, timed."""
    if STATE["abort"]:
        return False
    emit("c", name=name, **args)
    try:
        result = call()
        emit("c", name=name + "_done", result=str(getattr(result, "result", result))[:120].replace("\n", " "))
    except Exception as exc:
        emit("c", name=name + "_failed", error=("%s: %s" % (type(exc).__name__, exc))[:160])
        return False
    time.sleep(0.5)
    return True


def sequence(robot):
    b = robot.behavior
    emit("c", name="begin", control=has_control(robot), on_charger=bool(robot.status.is_on_charger))
    time.sleep(1.0)
    for angle in (0.0, 30.0, -15.0, 0.0):
        step(robot, "head", lambda a=angle: b.set_head_angle(degrees(a)), deg=angle)
    for height in (1.0, 0.0):
        step(robot, "lift", lambda h=height: b.set_lift_height(h), norm=height)
    if robot.status.is_on_charger:
        step(robot, "drive_off_charger", b.drive_off_charger)
        time.sleep(1.0)
    for speed, seconds in ((30.0, 2.5), (60.0, 2.5), (100.0, 1.5)):
        if not wheels_for(robot, -speed, speed, seconds, "spin_ccw_%d" % speed):
            break
        if not wheels_for(robot, speed, -speed, seconds, "spin_cw_%d" % speed):
            break
    for speed, seconds in ((40.0, 0.625), (80.0, 0.3125)):
        if not wheels_for(robot, -speed, -speed, seconds, "back_%d" % speed):
            break
        if not wheels_for(robot, speed, speed, seconds, "forward_%d" % speed):
            break
    emit("c", name="motion_end", abort=STATE["abort"])
    if has_control(robot) and not robot.status.is_on_charger and STATE["abort"] is None:
        emit("c", name="drive_on_charger")
        try:
            result = b.drive_on_charger()
            emit("c", name="drive_on_charger_done", result=str(getattr(result, "result", result))[:120].replace("\n", " "))
        except Exception as exc:
            emit("c", name="drive_on_charger_failed", error=("%s: %s" % (type(exc).__name__, exc))[:160])
    time.sleep(1.5)
    emit("c", name="end", on_charger=bool(robot.status.is_on_charger), control=has_control(robot))


def on_signal(signum, frame):
    robot = STATE["robot"]
    STATE["abort"] = "signal_%d" % signum
    if robot is not None:
        stop(robot, "signal")
    raise SystemExit(4)


def main():
    try:
        fd = os.open(LOCK, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    except FileExistsError:
        emit("c", name="lock_held", path=LOCK)
        return 3
    os.write(fd, ("a lane hardware recording pid %d\n" % os.getpid()).encode())
    os.close(fd)
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    robot = anki_vector.Robot(SERIAL, behavior_control_level=ControlPriorityLevel.DEFAULT_PRIORITY, cache_animation_lists=False)
    try:
        robot.connect(timeout=20)
        STATE["robot"] = robot
        robot.events.subscribe(on_state, Events.robot_state)
        try:
            sequence(robot)
        finally:
            stop(robot, "final")
            robot.events.unsubscribe(on_state, Events.robot_state)
            robot.disconnect()
    finally:
        os.remove(LOCK)
        emit("c", name="lock_released")
    return 0


if __name__ == "__main__":
    sys.exit(main())
