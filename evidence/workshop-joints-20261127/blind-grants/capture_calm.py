"""Capture a docked grant without any actuator commands."""
import csv
import math
from concurrent.futures import Future
from pathlib import Path
import sys
import time

import anki_vector as av
from anki_vector.events import Events


def capture(serial,ip,path):
    with path.open("x",buffering=1) as stream, av.Robot(serial,ip=ip,behavior_control_level=None) as robot:
        w=csv.writer(stream)
        w.writerow("receipt_s,event,head_deg,gyro_y,lift_mm,left_mm_s,right_mm_s,status".split(","))
        start=time.monotonic()
        def mark(event):
            w.writerow([time.monotonic()-start,event])
        calm_since = [None]
        def sample(r,event,m):
            if m.status & 1024 and r.status.is_on_charger:
                if calm_since[0] is None: calm_since[0]=time.monotonic()
            else: calm_since[0]=None
            w.writerow([time.monotonic()-start,"state",math.degrees(m.head_angle_rad),m.gyro.y,
                        m.lift_height_mm,m.left_wheel_speed_mmps,m.right_wheel_speed_mmps,m.status])
        robot.events.subscribe(sample,Events.robot_state)
        deadline=time.monotonic()+95
        while time.monotonic()<deadline:
            since=calm_since[0]
            if since is not None and time.monotonic()-since>=3: break
            time.sleep(.1)
        else:
            mark("calm_timeout")
            return
        b=robot.get_battery_state()
        assert b.is_on_charger_platform and b.battery_volts>=3.5
        mark("request")
        f=robot.conn.request_control(av.connection.ControlPriorityLevel.OVERRIDE_BEHAVIORS_PRIORITY,timeout=8)
        if isinstance(f,Future): f.result(timeout=10)
        mark("granted")
        time.sleep(8)
        b=robot.get_battery_state()
        assert b.is_on_charger_platform
        print(path.name,b.is_on_charger_platform,b.is_charging,b.battery_volts,flush=True)
        mark("disconnecting")


if __name__=="__main__":
    capture(sys.argv[1],sys.argv[2],Path(sys.argv[3]))
