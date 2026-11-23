"""Run the same head-only sequence on a docked Vector with an SDK configuration."""
import json
import math
import os
import sys
import time

import anki_vector as av
from anki_vector.events import Events
from anki_vector.util import degrees, radians


def main(serial, address):
    start = time.monotonic()
    def record(kind, **fields):
        print(json.dumps(dict(k=kind,t=time.monotonic()-start,**fields)),flush=True)

    def state(robot, event, msg):
        record("s",head=msg.head_angle_rad,lift=msg.lift_height_mm,gyro=[msg.gyro.x,msg.gyro.y,msg.gyro.z],
               wl=msg.left_wheel_speed_mmps,wr=msg.right_wheel_speed_mmps,status=msg.status,
               image_ms=msg.last_image_time_stamp,body_pitch=msg.pose_pitch_rad)

    lock = "/tmp/vector-"+serial+"-takeover.lock"
    fd = os.open(lock, os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
    try:
        priority = av.connection.ControlPriorityLevel.OVERRIDE_BEHAVIORS_PRIORITY
        with av.Robot(serial,ip=address,behavior_control_level=priority,behavior_activation_timeout=8) as robot:
            def check():
                b = robot.get_battery_state()
                if not b.is_on_charger_platform or robot.status.is_cliff_detected or robot.status.is_picked_up:
                    raise RuntimeError("Dock or support state changed")
                return b
            check()
            original = robot.head_angle_rad
            robot.events.subscribe(state,Events.robot_state)
            try:
                time.sleep(.8)
                for cycle in range(3):
                    for target in (40,-18,15,-8):
                        check()
                        record("command",cycle=cycle+1,target=target)
                        result = robot.behavior.set_head_angle(degrees(target)).result.code
                        record("done",code=result)
                        if result: raise RuntimeError("Head action failed")
                        time.sleep(.55)
            finally:
                check()
                restored = robot.behavior.set_head_angle(radians(original)).result.code
                time.sleep(.65)
                battery = check()
                record("end",restore_code=restored,original_deg=math.degrees(original),
                       final_deg=math.degrees(robot.head_angle_rad),volts=battery.battery_volts,
                       docked=battery.is_on_charger_platform,charging=battery.is_charging)
    finally:
        os.close(fd)
        os.unlink(lock)


if __name__ == "__main__":
    main(*sys.argv[1:])
