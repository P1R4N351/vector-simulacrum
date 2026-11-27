import csv, math, sys, time
from pathlib import Path
import anki_vector as av
from anki_vector.events import Events
from anki_vector.util import degrees

serial,ip,mode=sys.argv[1:]
active=mode=="active"
clock=time.monotonic
start=clock()
with open("/tmp/"+serial+"-docked-pose-new.csv","w",buffering=1) as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("s,event,condition,target,head_deg,lift_mm,gx,gy,gz,ax,ay,az,wl,wr,status,image_stamp".split(","))
    def state(r,n,m):
        w.writerow([clock()-start,"state","","",math.degrees(m.head_angle_rad),m.lift_height_mm,
                    m.gyro.x,m.gyro.y,m.gyro.z,m.accel.x,m.accel.y,m.accel.z,
                    m.left_wheel_speed_mmps,m.right_wheel_speed_mmps,m.status,m.last_image_time_stamp])
    with av.Robot(serial,ip=ip,behavior_control_level=av.connection.ControlPriorityLevel.OVERRIDE_BEHAVIORS_PRIORITY if active else None,behavior_activation_timeout=6) as r:
        r.events.subscribe(state,Events.robot_state)
        time.sleep(2)
        b=r.get_battery_state()
        print("start",b.is_on_charger_platform,b.is_charging,b.battery_volts,flush=True)
        if active:
            initial=math.degrees(r.head_angle_rad)
            assert b.is_on_charger_platform and b.is_charging and b.battery_volts>3.6
            initial_lift=r.lift_height_mm
            assert initial_lift<36,initial_lift
            try:
                for h in [0,.55,.55,0]:
                    assert r.status.is_on_charger
                    w.writerow([clock()-start,"lift",h,h])
                    action=r.behavior.set_lift_height(h)
                    assert action.result.code==0,action.result.code
                    time.sleep(.8)
                    for target in [-8,22,-8]:
                        assert r.status.is_on_charger
                        w.writerow([clock()-start,"head",h,target])
                        action=r.behavior.set_head_angle(degrees(target))
                        assert action.result.code==0,action.result.code
                        time.sleep(.8)
            finally:
                if r.status.is_on_charger:
                    r.behavior.set_lift_height(0)
                    r.behavior.set_head_angle(degrees(initial))
        else: time.sleep(32)
        b=r.get_battery_state()
        print("end",b.is_on_charger_platform,b.is_charging,b.battery_volts,flush=True)
