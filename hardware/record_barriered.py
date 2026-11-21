"""Second recording of Vector for the simulacrum: the sequence a BARRIERED surface allows.

Authored-by: CPCS (cpcs-5f62f24). Runs on the wire-pod host with its anki_vector install,
fed on stdin; writes JSON lines to stdout and leaves nothing behind but a lock file it removes.

First run on a robot 2026-11-18 (evidence/hw-rec-20261118). This copy is kept in
skills/vector-simulacrum so the sequence the model needs is written down beside the model. Nothing
in the skill imports it; tests/test_recorder.py runs it against a stand-in SDK with a stand-in clock.
Edited by cpcs-66ffc23, logging only, from the original with sha256
192269145c74c5eb3e5f523a232d845cac9ef769702426a1dc770afcefad8602:
after a hazard it keeps recording for ABORT_WATCH_S without commanding anything, the edge
phase needs and records EDGE_START_MM, and NOTE is copied into the begin row.

DO NOT RUN ON AN OPEN-EDGED TABLE. The straight runs need up to 450 mm of clear travel
(phase "straights" at 200 mm/s for 2.0 s, plus stopping), and the fast spins and arcs can
walk the robot sideways. The surface needs a wall, or a soft landing, on every side.

Choose phases with the environment variable PHASES (comma separated, default "rest,headlift,spins"):
  rest       stand still for 20 s, awake                              no travel
  headlift   head and lift over their whole range, and part moves     no travel
  spins      spins for 3.0 s both ways at 150 and 220 mm/s            turns on the spot; allow 100 mm all round
  straights  forward then back for 2.0 s at 50, 100, 150, 200 mm/s    up to 450 mm forward of the start
  reversal   forward 100 mm/s for 1.0 s straight into back 1.0 s      about 120 mm forward
  arcs       left 60 / right 120 mm/s for 3.0 s, then mirrored        a circle of about 150 mm radius
  actions    drive_straight 100 mm at 50 and 100 mm/s; turn_in_place  about 120 mm forward
             90 degrees at 60 deg/s both ways
  repeat     one 60 mm/s spin and one 80 mm/s step, as in run 1       about 30 mm; put it LAST in PHASES
  edge       ONE approach to an edge at EDGE_SPEED mm/s (default 30)  drives until the robot's own cliff
             for at most EDGE_SECONDS (default 3.0)                    reaction stops it
  wheelstand head level, lift down, then LISTEN ONLY for WHEELSTAND_S   no travel; a person tips the robot
             seconds (default 180): nothing is commanded while a       by hand onto its rear wheels and onto
             person tips the robot (simulacrum/wheelstand.py)          one tread edge; a soft surface
  wheelie    connect the light cube, then the robot's own              drives to the cube and tips onto its
             pop_a_wheelie off it; records WHEELIE_WATCH_S (default    rear; it finds the cube with its camera,
             8) after; must be LAST in PHASES                          so the room must be lit
  endurance  sustained load: spins both ways at ENDURANCE_MMPS          turns on the spot; allow 100 mm all
             (default 300, above the treads' top speed) for 3 s, 2 s   round; drains the battery
             rest, repeated for ENDURANCE_S (default 600, at most
             1800); stops at 3.6 V
  liftspins  head level; spins 3 s both ways at 150 mm/s with the lift  turns on the spot; allow 100 mm all
             down, then up, then down again; lift left down             round
  nudge      short moves back and forth at 40 mm/s to seat a robot      under 25 mm each way
             on its charger ramp; stops on charger contact
  dock       the robot's own drive_on_charger; must be LAST, not with  drives to its charger, which it finds
             wheelie. PHASES=dock alone on the charger commands         with its camera
             nothing (it is not driven off first)

The edge phase. Before the run, with the robot standing square to the edge, measure with a ruler
from the line where the front wheels touch the surface to the edge, and pass it as EDGE_START_MM.
The phase refuses to start without it. Nothing can be measured by hand afterwards: about 1.2 s
after it stops the robot backs away by itself and may turn. The stop point is the start distance
less the travel the robot reports. Start between EDGE_SPEED x 1.0 s and EDGE_SPEED x
(EDGE_SECONDS - 0.5 s) from the edge, so the robot is at speed when it arrives and arrives in time.

ABORT_WATCH_S (default 6): after a cliff or any other hazard ends the run, the script sends its
stop and then keeps recording for this long without commanding anything, so the robot's own
reaction (the pause, the move away, the turn) is in the file.

NOTE: free text copied into the begin row (the surface, the barrier, ruler readings).

img_ts is the robot's time stamp of its last camera image. It shows how evenly state messages
arrive. It cannot split the delay of a command going in from the delay of state coming back:
the two clocks share no event.

Battery: a "b" row (volts, level, charging, on_platform) at every phase start, at the end, and before each
endurance cycle, read with get_battery_state while the wheels are stopped. Never inside the hazard loop:
it is a blocking call, and in run F (2026-11-18) it returned 4.0797 V unchanged 165 times in 2.5 min of
full-power spins, so it carries no second-by-second sag.

RULER_PAUSE_S (default 0): seconds to stand still after each forward leg of "straights", so the
distance can be marked and measured before the robot drives back.

Safety:
- Control is requested at DEFAULT priority, so the robot's own cliff reaction stays on and
  can take control away. If control is not granted at the start (it is refused while the
  cliff flag is set), the script exits without commanding anything.
- Every motion is bounded in time and followed by stop_all_motors.
- A cliff, pick-up, fall or held flag, or loss of control, ends all motion for the run.
  The script then only listens (ABORT_WATCH_S) and commands nothing more.
- A blocking behaviour call (head, lift, drive_straight, turn_in_place, drive_off_charger) has
  no time limit of its own here. It is not started without control, but if control is lost
  while it runs it may not return: end the script with SIGTERM, which stops the motors. SIGHUP
  (an ssh connection dropping) and SIGINT stop them too; SIGKILL cannot be caught.
- VECTOR_SERIAL picks the robot (8 hex digits; default the robot of the first recording).
- VECTOR_IP, if set, is the robot's address, overriding the SDK config (for a robot on a daughter pod).
- It does not try to dock. The robot is left where the sequence ends.
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
from anki_vector.util import degrees, distance_mm, speed_mmps

SERIAL = os.environ.get("VECTOR_SERIAL", "00000000")
assert len(SERIAL) == 8 and all(c in "0123456789abcdef" for c in SERIAL.lower()), "VECTOR_SERIAL is 8 hex digits"
# VECTOR_IP: the robot's address when the SDK config's is out of date (a robot behind a daughter pod).
IP = os.environ.get("VECTOR_IP", "")
assert IP == "" or (IP.count(".") == 3 and all(p.isdigit() and 0 <= int(p) <= 255 for p in IP.split("."))), \
    "VECTOR_IP is a dotted IPv4 address"
LOCK = "/tmp/vector-%s-takeover.lock" % SERIAL
POLL_S = 0.02
MAX_WHEEL_MMPS = 220.0
MAX_SEGMENT_S = 3.5
MAX_WATCH_S = 30.0
MAX_LISTEN_S = 300.0
# SIGHUP too: run over ssh, a dropped connection hangs up the script, and wheels set with
# set_wheel_motors keep turning after the process is gone unless it stops them first.
STOP_SIGNALS = (signal.SIGTERM, signal.SIGINT, signal.SIGHUP)
CUBE_WAIT_S = 12.0
CUBE_POLL_S = 0.5
KNOWN_PHASES = ("rest", "headlift", "spins", "straights", "reversal", "arcs", "actions", "repeat", "edge", "wheelstand",
                "wheelie", "dock", "endurance", "liftspins", "nudge")
# The SDK's get_battery_state: "At 3.6V, the robot is approaching low charge" (anki_vector/robot.py:765).
ENDURANCE_LOW_V = 3.6
ENDURANCE_SPIN_S = 3.0
ENDURANCE_REST_S = 2.0
MAX_ENDURANCE_MMPS = 300.0
MAX_ENDURANCE_S = 1800.0
OUT_LOCK = threading.Lock()
T0 = time.monotonic()
STATE = {"robot": None, "abort": None, "volts": None}


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
    """One battery reading as a "b" row; the voltage is kept for the endurance phase's low-battery stop."""
    try:
        b = robot.get_battery_state()
        volts = float(b.battery_volts)
    except Exception as exc:  # a missed reading is recorded; the run goes on
        emit("b", error=type(exc).__name__)
        return
    STATE["volts"] = volts
    emit("b", volts=round(volts, 4), level=int(b.battery_level), charging=bool(b.is_charging),
         on_platform=bool(b.is_on_charger_platform))


def has_control(robot):
    """Control as the connection's control_granted_event says. In wirepod_vector_sdk 0.8.1 the
    has_control flag is not cleared when control is lost, so it alone would miss a cliff reaction."""
    granted = getattr(robot.conn, "control_granted_event", None)
    if granted is not None:
        return bool(granted.is_set())
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
    issued_t = round(time.monotonic() - T0, 4)
    try:
        robot.motors.stop_all_motors()
    except Exception as exc:  # best effort; the reason is still recorded
        emit("c", name="stop_failed", why=why, error=type(exc).__name__)
        return
    emit("c", name="stop", why=why, issued_t=issued_t)


def clear_to_move(robot, label):
    """False once the run has been aborted or a hazard shows; records why."""
    if STATE["abort"]:
        return False
    reason = hazard(robot)
    if reason:
        STATE["abort"] = reason
        emit("c", name="skip", label=label, why=reason)
        return False
    return True


def hold(robot, seconds):
    """Keep the present wheel command for a bounded time. False if a hazard ended it."""
    assert 0.0 < seconds <= MAX_SEGMENT_S, "segment too long"
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        reason = hazard(robot)
        if reason:
            STATE["abort"] = reason
            stop(robot, reason)
            return False
        time.sleep(POLL_S)
    return True


def wheels_for(robot, left, right, seconds, label, settle_s=2.0, then=None, limit=MAX_WHEEL_MMPS):
    """Two wheel speeds for a bounded time, optionally straight into a second pair, then stop and stand."""
    assert abs(left) <= limit and abs(right) <= limit <= MAX_ENDURANCE_MMPS, "wheel speed above the limit"
    if not clear_to_move(robot, label):
        return False
    emit("c", name="wheels", label=label, left=left, right=right, seconds=seconds)
    robot.motors.set_wheel_motors(left, right)
    emit("c", name="wheels_sent", label=label)
    if not hold(robot, seconds):
        return False
    if then is not None:
        left2, right2, seconds2 = then
        assert abs(left2) <= MAX_WHEEL_MMPS and abs(right2) <= MAX_WHEEL_MMPS
        emit("c", name="wheels", label=label + "_then", left=left2, right=right2, seconds=seconds2)
        robot.motors.set_wheel_motors(left2, right2)
        emit("c", name="wheels_sent", label=label + "_then")
        if not hold(robot, seconds2):
            return False
    stop(robot, "segment_end")
    time.sleep(settle_s)
    return True


def behaviour(robot, name, call, **args):
    """One blocking SDK behaviour call, timed. Not started without control: it would block."""
    if not clear_to_move(robot, name):
        return False
    emit("c", name=name, **args)
    try:
        result = call()
        emit("c", name=name + "_done", result=str(getattr(result, "result", result))[:120].replace("\n", " "))
    except Exception as exc:
        emit("c", name=name + "_failed", error=("%s: %s" % (type(exc).__name__, exc))[:160])
        return False
    time.sleep(2.0)
    return True


def phase_rest(robot):
    emit("c", name="rest", seconds=20.0)
    time.sleep(20.0)


def phase_headlift(robot):
    b = robot.behavior
    for angle in (-22.0, 45.0, -22.0, 45.0, -22.0, 10.0, -22.0, 0.0):
        behaviour(robot, "head", lambda a=angle: b.set_head_angle(degrees(a)), deg=angle)
    for height in (0.0, 1.0, 0.0, 1.0, 0.0, 0.4667, 0.0):  # 0.4667 of the 32..92 mm range is 60 mm
        behaviour(robot, "lift", lambda h=height: b.set_lift_height(h), norm=height)


def phase_spins(robot):
    for speed in (150.0, 220.0):
        if not wheels_for(robot, -speed, speed, 3.0, "spin_ccw_%d" % speed):
            return
        if not wheels_for(robot, speed, -speed, 3.0, "spin_cw_%d" % speed):
            return


NUDGE_MMPS = 40.0
NUDGE_TRIES = 3


def phase_nudge(robot):
    """Seat a robot that sits on its charger's ramp without contact (the operator: "drive it back and forth it'll fix
    itself"). Up to NUDGE_TRIES pairs of short moves at NUDGE_MMPS, back then forward (Vector docks in
    reverse), each a little longer than the last, ending with a final move back; stops as soon as the
    charger contact shows. Records charger contact after every move."""
    if robot.status.is_on_charger:
        emit("c", name="already_on_charger")
        return
    for attempt in range(NUDGE_TRIES):
        seconds = 0.4 + 0.1 * attempt
        for way, speed in (("back", -NUDGE_MMPS), ("forward", NUDGE_MMPS)):
            if not wheels_for(robot, speed, speed, seconds, "nudge_%s_%d" % (way, attempt), settle_s=0.5):
                return
            emit("c", name="nudge_contact", on_charger=bool(robot.status.is_on_charger))
            if robot.status.is_on_charger:
                return
    if wheels_for(robot, -NUDGE_MMPS, -NUDGE_MMPS, 0.6, "nudge_back_final", settle_s=0.5):
        emit("c", name="nudge_contact", on_charger=bool(robot.status.is_on_charger))


def phase_liftspins(robot):
    """Spins at 150 mm/s both ways with the lift down, then up, then down again (A-B-A, so a slow change
    of battery or heat over the phase does not read as the lift's effect). Head level throughout; lift
    left down at the end."""
    b = robot.behavior
    if not behaviour(robot, "head", lambda: b.set_head_angle(degrees(0.0)), deg=0.0):
        return
    for tag, norm in (("down_a", 0.0), ("up", 1.0), ("down_b", 0.0)):
        if not behaviour(robot, "lift", lambda h=norm: b.set_lift_height(h), norm=norm):
            return
        for way, left, right in (("ccw", -150.0, 150.0), ("cw", 150.0, -150.0)):
            if not wheels_for(robot, left, right, 3.0, "liftspin_%s_%s" % (tag, way)):
                return
    behaviour(robot, "lift", lambda: b.set_lift_height(0.0), norm=0.0)


def phase_straights(robot):
    pause = float(os.environ.get("RULER_PAUSE_S", "0") or 0.0)
    assert 0.0 <= pause <= 120.0, "RULER_PAUSE_S out of range"
    for speed in (50.0, 100.0, 150.0, 200.0):
        if not wheels_for(robot, speed, speed, 2.0, "forward_%d" % speed):
            return
        if pause > 0.0:
            emit("c", name="ruler_pause", seconds=pause, after="forward_%d" % speed)
            time.sleep(pause)
        if not wheels_for(robot, -speed, -speed, 2.0, "back_%d" % speed):
            return


def phase_reversal(robot):
    wheels_for(robot, 100.0, 100.0, 1.0, "reversal_100", then=(-100.0, -100.0, 1.0))


def phase_arcs(robot):
    if wheels_for(robot, 60.0, 120.0, 3.0, "arc_left_60_120"):
        wheels_for(robot, 120.0, 60.0, 3.0, "arc_right_120_60")


def phase_actions(robot):
    b = robot.behavior
    for speed in (50.0, 100.0):
        behaviour(robot, "drive_straight", lambda s=speed: b.drive_straight(distance_mm(100.0), speed_mmps(s), should_play_anim=False), mm=100.0, mmps=speed)
        behaviour(robot, "drive_straight", lambda s=speed: b.drive_straight(distance_mm(-100.0), speed_mmps(s), should_play_anim=False), mm=-100.0, mmps=speed)
    for angle in (90.0, -90.0):
        behaviour(robot, "turn_in_place", lambda a=angle: b.turn_in_place(degrees(a), speed=degrees(60.0)), deg=angle, dps=60.0)


def phase_repeat(robot):
    if wheels_for(robot, -60.0, 60.0, 2.5, "repeat_spin_ccw_60"):
        if wheels_for(robot, -80.0, -80.0, 0.3125, "repeat_back_80"):
            wheels_for(robot, 80.0, 80.0, 0.3125, "repeat_forward_80")


def edge_settings():
    """Speed, time limit and the ruler-measured start distance of the edge phase. Raises if any is missing or out of range."""
    speed = float(os.environ.get("EDGE_SPEED", "30") or 30.0)
    seconds = float(os.environ.get("EDGE_SECONDS", "3.0") or 3.0)
    start = float(os.environ.get("EDGE_START_MM", "0") or 0.0)
    assert 10.0 <= speed <= 140.0, "EDGE_SPEED must be 10..140 mm/s"
    assert 0.5 <= seconds <= MAX_SEGMENT_S, "EDGE_SECONDS out of range"
    assert 5.0 <= start <= speed * seconds, "EDGE_START_MM (front wheel contact line to the edge, by ruler) is missing or out of reach"
    return speed, seconds, start


def phase_edge(robot):
    """One approach. The robot's own reaction is what stops it; this only bounds the time."""
    speed, seconds, start = edge_settings()
    emit("c", name="edge_setup", speed=speed, seconds=seconds, start_to_edge_mm=start)
    wheels_for(robot, speed, speed, seconds, "edge_approach_%d" % speed, settle_s=4.0)


def wheelstand_seconds():
    """How long the wheelstand phase listens. Raises if out of range."""
    seconds = float(os.environ.get("WHEELSTAND_S", "180") or 0.0)
    assert 10.0 <= seconds <= MAX_LISTEN_S, "WHEELSTAND_S must be 10..300 s"
    return seconds


def phase_wheelstand(robot):
    """Head level and lift down (the pose the table's centre of mass is for), then command nothing and listen.

    The person tips the robot by hand; the state rows carry the accelerometer, head and lift.
    Hazard flags (picked up, held, cliff) are expected here and are not acted on, because nothing moves.
    Whether the firmware moves the head or lift itself while the robot is held is not known; the
    analysis reads head and lift from every row and sets aside any trial off the pose.
    """
    seconds = wheelstand_seconds()
    b = robot.behavior
    if not behaviour(robot, "head", lambda: b.set_head_angle(degrees(0.0)), deg=0.0):
        return
    if not behaviour(robot, "lift", lambda: b.set_lift_height(0.0), norm=0.0):
        return
    emit("c", name="wheelstand_listen", seconds=seconds)
    time.sleep(seconds)
    emit("c", name="wheelstand_listen_done")


def wheelie_watch_seconds():
    seconds = float(os.environ.get("WHEELIE_WATCH_S", "8") or 0.0)
    assert 0.0 <= seconds <= MAX_WATCH_S, "WHEELIE_WATCH_S out of range"
    return seconds


def _pose_fields(pose):
    """x, y, z in mm and heading in degrees of an SDK pose, or None if it cannot be read."""
    try:
        p = pose.position
        return [round(float(p.x), 1), round(float(p.y), 1), round(float(p.z), 1), round(float(pose.rotation.angle_z.degrees), 2)]
    except Exception as exc:  # the pose is evidence only; its absence is recorded, not fatal
        return "unreadable: %s" % type(exc).__name__


def _connect_fields(reply):
    """What the robot answered to ConnectCube: success, the cube's radio id, the response status."""
    fields = {}
    for name in ("success", "factory_id", "object_id"):
        value = getattr(reply, name, None)
        fields[name] = value if isinstance(value, (bool, int, float, str)) or value is None else str(value)
    status = getattr(reply, "status", None)
    fields["status_code"] = getattr(status, "code", None) if status is not None else None
    if fields["status_code"] is not None and not isinstance(fields["status_code"], int):
        fields["status_code"] = str(fields["status_code"])
    return fields


def _connected_cube(robot):
    """Connect the light cube and wait for it, at most CUBE_WAIT_S. None if it never connects."""
    try:
        reply = robot.world.connect_cube()
    except Exception as exc:
        emit("c", name="cube_connect_failed", error=("%s: %s" % (type(exc).__name__, exc))[:160])
        return None
    emit("c", name="cube_connect", **_connect_fields(reply))
    for _ in range(int(CUBE_WAIT_S / CUBE_POLL_S)):
        cube = robot.world.connected_light_cube
        if cube is not None:
            return cube
        time.sleep(CUBE_POLL_S)
    emit("c", name="no_cube", waited_s=CUBE_WAIT_S)
    return None


def phase_wheelie(robot):
    """The robot's own wheelie: it drives to its cube, presses its lift on it and tips onto its rear.

    This script only starts the behaviour; the firmware drives. The cube's and the robot's poses
    (the robot's own estimates) are recorded before, so a model can put a cube in the same place.
    Picked-up and falling flags are expected during it, so it must be the last phase. Nothing is
    commanded after it; the robot's own return to its treads is recorded for WHEELIE_WATCH_S.
    """
    watch = wheelie_watch_seconds()
    cube = _connected_cube(robot)
    if cube is None:
        return
    emit("c", name="wheelie_setup", cube_pose=_pose_fields(cube.pose), robot_pose=_pose_fields(robot.pose))
    behaviour(robot, "pop_a_wheelie", lambda: robot.behavior.pop_a_wheelie(cube))
    emit("c", name="wheelie_watch", seconds=watch)
    time.sleep(watch)
    try:
        robot.world.disconnect_cube()
    except Exception as exc:  # best effort; the cube disconnects on its own when the SDK does
        emit("c", name="cube_disconnect_failed", error=type(exc).__name__)


def phase_dock(robot):
    """The robot's own return to its charger. Records whether it ends on the charger."""
    if robot.status.is_on_charger:
        emit("c", name="already_on_charger")
        return
    behaviour(robot, "drive_on_charger", robot.behavior.drive_on_charger)
    emit("c", name="dock_result", on_charger=bool(robot.status.is_on_charger))


def endurance_settings():
    """ENDURANCE_S and ENDURANCE_MMPS, checked before connecting."""
    seconds = float(os.environ.get("ENDURANCE_S", "600") or 0.0)
    speed = float(os.environ.get("ENDURANCE_MMPS", "300") or 0.0)
    assert 60.0 <= seconds <= MAX_ENDURANCE_S, "ENDURANCE_S must be 60..1800 s"
    assert 100.0 <= speed <= MAX_ENDURANCE_MMPS, "ENDURANCE_MMPS must be 100..300 mm/s"
    return seconds, speed


def phase_endurance(robot):
    """Sustained load: spins both ways, ENDURANCE_SPIN_S each with ENDURANCE_REST_S between, for ENDURANCE_S.

    Commanded above the treads' top speed, so the speed they reach is the ceiling at that moment's
    battery voltage, read once a second. Stops early at ENDURANCE_LOW_V.
    """
    seconds, speed = endurance_settings()
    end = time.monotonic() + seconds
    cycles = int(seconds / (2.0 * (ENDURANCE_SPIN_S + ENDURANCE_REST_S))) + 1
    done = 0
    for cycle in range(cycles):
        if time.monotonic() >= end:
            break
        battery_read(robot)
        if STATE["volts"] is not None and STATE["volts"] <= ENDURANCE_LOW_V:
            emit("c", name="endurance_low_battery", volts=STATE["volts"])
            break
        for sign, way in ((1.0, "ccw"), (-1.0, "cw")):
            if not wheels_for(robot, -sign * speed, sign * speed, ENDURANCE_SPIN_S, "endurance_%s_%d" % (way, cycle),
                              settle_s=0.0, limit=MAX_ENDURANCE_MMPS) or not hold(robot, ENDURANCE_REST_S):
                return
        done += 1
    emit("c", name="endurance_done", cycles=done, seconds=seconds, mmps=speed, volts=STATE["volts"])


def watch_after_abort():
    """After a hazard: command nothing, keep recording, so the robot's own reaction is in the file."""
    seconds = float(os.environ.get("ABORT_WATCH_S", "6") or 0.0)
    assert 0.0 <= seconds <= MAX_WATCH_S, "ABORT_WATCH_S out of range"
    emit("c", name="abort_watch", seconds=seconds, why=STATE["abort"])
    time.sleep(seconds)


PHASES = {"rest": phase_rest, "headlift": phase_headlift, "spins": phase_spins, "straights": phase_straights,
          "reversal": phase_reversal, "arcs": phase_arcs, "actions": phase_actions, "repeat": phase_repeat, "edge": phase_edge,
          "wheelstand": phase_wheelstand, "wheelie": phase_wheelie, "dock": phase_dock,
          "endurance": phase_endurance, "liftspins": phase_liftspins, "nudge": phase_nudge}


def chosen_phases():
    names = [n.strip() for n in os.environ.get("PHASES", "rest,headlift,spins").split(",") if n.strip()]
    unknown = [n for n in names if n not in KNOWN_PHASES]
    assert not unknown, "unknown phases: %s" % unknown
    assert names, "no phases chosen"
    if "edge" in names:
        edge_settings()  # refuse before connecting, not half way through the run
    if "wheelstand" in names:
        wheelstand_seconds()
    if "wheelie" in names:
        assert names[-1] == "wheelie" and "repeat" not in names, "wheelie must be the last phase, and not with repeat"
        wheelie_watch_seconds()
    if "endurance" in names:
        endurance_settings()
    if "dock" in names:
        assert names[-1] == "dock" and "wheelie" not in names, "dock must be the last phase, and not with wheelie"
    return names


def sequence(robot, names):
    st = robot.status
    # utc: the host's wall clock at this row, so other recordings (a camera) can be aligned with t.
    emit("c", name="begin", phases=names, control=has_control(robot), on_charger=bool(st.is_on_charger),
         cliff=bool(st.is_cliff_detected), note=os.environ.get("NOTE", "")[:400],
         utc=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds"))
    if not has_control(robot):
        emit("c", name="no_control", why="not granted at DEFAULT priority; nothing was commanded")
        return
    if st.is_on_charger and names not in (["dock"], ["nudge"]):
        if not behaviour(robot, "drive_off_charger", robot.behavior.drive_off_charger):
            return
    for name in names:
        if STATE["abort"]:
            break
        emit("c", name="phase", phase=name)
        battery_read(robot)
        PHASES[name](robot)
    if STATE["abort"]:
        watch_after_abort()
    battery_read(robot)
    emit("c", name="end", abort=STATE["abort"], control=has_control(robot), on_charger=bool(robot.status.is_on_charger))


def on_signal(signum, frame):
    robot = STATE["robot"]
    STATE["abort"] = "signal_%d" % signum
    if robot is not None:
        stop(robot, "signal")
    raise SystemExit(4)


def main():
    names = chosen_phases()
    try:
        fd = os.open(LOCK, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    except FileExistsError:
        emit("c", name="lock_held", path=LOCK)
        return 3
    os.write(fd, ("cpcs-5f62f24 hardware recording pid %d\n" % os.getpid()).encode())
    os.close(fd)
    for sig in STOP_SIGNALS:
        signal.signal(sig, on_signal)
    extra = {"ip": IP} if IP else {}
    robot = anki_vector.Robot(SERIAL, behavior_control_level=ControlPriorityLevel.DEFAULT_PRIORITY,
                              cache_animation_lists=False, **extra)
    emit("c", name="connect_to", serial=SERIAL, ip=IP or "from the SDK config")
    try:
        robot.connect(timeout=20)
        STATE["robot"] = robot
        robot.events.subscribe(on_state, Events.robot_state)
        try:
            time.sleep(1.0)
            sequence(robot, names)
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
