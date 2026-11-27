"""Record a docked lift-duration crossover. Invoke with serial, address, output CSV."""
import csv
import math
import sys
import time

import anki_vector as av
from anki_vector.events import Events

serial, address, path = sys.argv[1:]
with open(path, "x", buffering=1) as file, av.Robot(
    serial, ip=address, behavior_control_level=av.connection.ControlPriorityLevel.OVERRIDE_BEHAVIORS_PRIORITY,
    behavior_activation_timeout=6,
) as robot:
    writer = csv.writer(file, lineterminator="\n")
    writer.writerow("s,event,target_mm,duration_s,lift_mm,head_deg,gyro_y,left_mmps,right_mmps,on_dock,code".split(","))
    start = time.monotonic()
    def state(r, _, msg):
        writer.writerow([time.monotonic()-start,"state","","",msg.lift_height_mm,math.degrees(msg.head_angle_rad),
                         msg.gyro.y,msg.left_wheel_speed_mmps,msg.right_wheel_speed_mmps,int(bool(msg.status & 4096)),""])
    robot.events.subscribe(state, Events.robot_state)
    time.sleep(1)
    battery = robot.get_battery_state()
    print("before", battery.is_on_charger_platform, battery.battery_volts, flush=True)
    assert battery.is_on_charger_platform and battery.battery_volts >= 3.5 and robot.lift_height_mm < 36
    def move(target, duration):
        assert robot.status.is_on_charger
        writer.writerow([time.monotonic()-start,"command",target,duration])
        reply = robot.behavior.set_lift_height((target-32)/60, duration=duration)
        writer.writerow([time.monotonic()-start,"return",target,duration,"","","","","","",reply.result.code])
        assert reply.result.code == 0
        time.sleep(.75)
    try:
        time.sleep(4.5)
        for low in [32,40,40,32,40,32,32,40]:
            move(low+12,0)
            move(low,.600)
    finally:
        if robot.status.is_on_charger and robot.lift_height_mm > 36:
            move(32,0)
    battery = robot.get_battery_state()
    print("after", battery.is_on_charger_platform, battery.battery_volts, flush=True)
