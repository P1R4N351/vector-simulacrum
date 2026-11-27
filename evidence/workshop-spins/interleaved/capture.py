import csv,sys,time
import anki_vector as av
from anki_vector.events import Events
from anki_vector.util import degrees,distance_mm,speed_mmps
clock=time.monotonic
start=clock()
phase='dock'
last=0
flags=0
def log(*args):
 print(*args,flush=True)
with open('/tmp/workshop-mixed-states.csv','w',buffering=1) as f, open('/tmp/workshop-mixed-events.csv','w',buffering=1) as e:
 w=csv.writer(f);m=csv.writer(e)
 w.writerow('time_s phase left_mm_s right_mm_s gyro_x gyro_y gyro_z head_rad lift_mm pitch_rad x_mm y_mm flags'.split())
 m.writerow(['time_s','event','detail'])
 def event(name,detail=''):
  m.writerow([clock()-start,name,detail]);log(name,detail)
 def state(r,n,s):
  global last,flags
  last=clock();flags=int(s.status)
  w.writerow([last-start,phase,s.left_wheel_speed_mmps,s.right_wheel_speed_mmps,s.gyro.x,s.gyro.y,s.gyro.z,s.head_angle_rad,s.lift_height_mm,s.pose_pitch_rad,s.pose.x,s.pose.y,flags])
 def guard():
  if clock()-last>.45 or flags & (8|32|16384|65536): raise RuntimeError('telemetry or footing')
 with av.Robot(sys.argv[1],ip=sys.argv[2],behavior_control_level=av.connection.ControlPriorityLevel.OVERRIDE_BEHAVIORS_PRIORITY) as r:
  r.events.subscribe(state,Events.robot_state)
  time.sleep(6)
  b=r.get_battery_state()
  assert b.is_on_charger_platform and b.battery_volts>=3.6
  try:
   phase='undock'
   event(phase)
   a=r.behavior.drive_off_charger()
   event('undock_result',str(a).strip())
   time.sleep(.4)
   assert not r.get_battery_state().is_on_charger_platform
   guard()
   phase='clear_dock'
   a=r.behavior.drive_straight(distance_mm(65),speed_mmps(40),should_play_anim=False)
   assert a.result.code==0,str(a)
   time.sleep(.5);guard()
   for i,(sign,speed) in enumerate([(1,100),(-1,60),(1,150),(-1,150),(1,60),(-1,100)]):
    phase='turn'+str(i+1)
    guard()
    event('spin',sign*speed)
    r.motors.set_wheel_motors(-speed*sign,speed*sign)
    stop=clock()+1.1
    try:
     while clock()<stop:
      guard();time.sleep(.025)
    finally:
     r.motors.set_wheel_motors(0,0);event('stop',sign)
    time.sleep(.85)
  finally:
   phase='return'
   r.motors.stop_all_motors()
   if not r.get_battery_state().is_on_charger_platform:
    guard()
    event('dock')
    a=r.behavior.drive_on_charger()
    event('dock_result',str(a).strip())
   time.sleep(1)
   b=r.get_battery_state();event('end',(b.is_on_charger_platform,b.is_charging,b.battery_volts))
