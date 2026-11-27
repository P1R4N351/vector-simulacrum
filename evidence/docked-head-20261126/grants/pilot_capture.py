import csv
import math
import os
import sys
import time
from concurrent.futures import Future

import anki_vector as av
from anki_vector.events import Events

serial, ip = sys.argv[1:3]
session = int(os.environ["GRANT_SESSION"])
with open(f"/tmp/grant-session-{session}.csv","w",buffering=1) as f:
    writer = csv.writer(f)
    writer.writerow("receipt_s,event,head_deg,gyro_y,lift_mm,left_mm_s,right_mm_s,status".split(","))
    start = time.monotonic()
    def mark(event):
        writer.writerow([time.monotonic()-start,event])
    def state(robot,name,m):
        writer.writerow([time.monotonic()-start,"state",math.degrees(m.head_angle_rad),m.gyro.y,
                         m.lift_height_mm,m.left_wheel_speed_mmps,m.right_wheel_speed_mmps,m.status])
    with av.Robot(serial, ip=ip, behavior_control_level=None) as robot:
        robot.events.subscribe(state,Events.robot_state)
        mark("subscribed")
        time.sleep([9,14][session-1])
        for i in range(2):
            b=robot.get_battery_state()
            assert b.is_on_charger_platform and b.battery_volts>=3.6
            print("battery",i,b.is_on_charger_platform,b.is_charging,b.battery_volts,flush=True)
            mark("request")
            granted=robot.conn.request_control(av.connection.ControlPriorityLevel.OVERRIDE_BEHAVIORS_PRIORITY,timeout=8)
            if isinstance(granted,Future): granted.result(timeout=10)
            mark("granted")
            time.sleep(8)
            assert robot.status.is_on_charger
            mark("release_request")
            released=robot.conn.release_control()
            if isinstance(released,Future): released.result(timeout=10)
            mark("released")
            time.sleep(10 if i==0 else 3)
        b=robot.get_battery_state()
        print("end",b.is_on_charger_platform,b.is_charging,b.battery_volts,flush=True)
