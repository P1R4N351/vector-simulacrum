"""Observe SDK state without acquiring behavior control."""
import csv
import sys
import time
import anki_vector as av
from anki_vector.events import Events

serial,ip=sys.argv[1:3]
start=time.monotonic()
with open("/tmp/"+serial+"-idle-cadence.csv","w",buffering=1) as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("s,gx,gy,gz,ax,ay,az,head_rad,lift_mm,wl,wr,status".split(","))
    def state(robot,event,m):
        w.writerow([time.monotonic()-start,m.gyro.x,m.gyro.y,m.gyro.z,m.accel.x,m.accel.y,m.accel.z,
                    m.head_angle_rad,m.lift_height_mm,m.left_wheel_speed_mmps,m.right_wheel_speed_mmps,m.status])
    with av.Robot(serial,ip=ip,behavior_control_level=None) as robot:
        robot.events.subscribe(state,Events.robot_state)
        b=robot.get_battery_state()
        assert b.is_on_charger_platform
        time.sleep(62)
        b=robot.get_battery_state()
        print("dock",b.is_on_charger_platform,"charging",b.is_charging,flush=True)
