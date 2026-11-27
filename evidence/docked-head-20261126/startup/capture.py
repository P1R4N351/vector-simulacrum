import os, csv, math, sys, time
import anki_vector as av
from anki_vector.events import Events
from anki_vector.util import degrees
serial,ip,mode=sys.argv[1:]
clock=time.monotonic
session = int(os.environ['HEAD_SESSION'])
wait_s = [1,9,9,1][session-1]
with open('/tmp/head-startup-'+str(session)+'.csv','w',buffering=1) as f:
 w=csv.writer(f)
 w.writerow('t_s,event,trial,target_deg,head_deg,gyro_y_rad_s,lift_mm,pitch_rad,left_mm_s,right_mm_s,status'.split(','))
 start=clock()
 def state(r,n,m):
  w.writerow([clock()-start,'state','','',math.degrees(m.head_angle_rad),m.gyro.y,m.lift_height_mm,m.pose_pitch_rad,m.left_wheel_speed_mmps,m.right_wheel_speed_mmps,m.status])
 priority=av.connection.ControlPriorityLevel.OVERRIDE_BEHAVIORS_PRIORITY if mode=='active' else None
 with av.Robot(serial,ip=ip,behavior_control_level=priority,behavior_activation_timeout=6) as r:
  r.events.subscribe(state,Events.robot_state)
  w.writerow([clock()-start,"subscribed"])
  time.sleep(wait_s)
  b=r.get_battery_state()
  print('start',b.is_on_charger_platform,b.is_charging,b.battery_volts,flush=True)
  if mode=='active':
   assert b.is_on_charger_platform and b.battery_volts>=3.6
   initial=math.degrees(r.head_angle_rad)
   try:
    for i,target in enumerate([-10,25,-10]):
     assert r.status.is_on_charger
     w.writerow([clock()-start,'command',i,target])
     result=r.behavior.set_head_angle(degrees(target))
     w.writerow([clock()-start,'return',i,target,result.result.code])
     if result.result.code:raise RuntimeError(result)
     time.sleep(2 if target>0 else .9)
   finally:
    if r.status.is_on_charger:r.behavior.set_head_angle(degrees(initial))
  else:time.sleep(8)
  b=r.get_battery_state()
  print('end',b.is_on_charger_platform,b.is_charging,b.battery_volts,flush=True)
