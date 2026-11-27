"""Docked head holds at two delays after override. Supply SDK serial, IP and output folder."""
import csv
import math
from concurrent.futures import Future
from pathlib import Path
import sys
import time

import anki_vector as av
from anki_vector.events import Events
from anki_vector.util import degrees


def session(serial, ip, path, delay):
    start = time.monotonic()
    with path.open("x", buffering=1) as file, av.Robot(serial, ip=ip, behavior_control_level=None) as robot:
        writer = csv.writer(file)
        writer.writerow("s,event,value,head_deg,gyro_y,wl,wr,status,on_dock".split(","))
        def mark(name,value=""):
            writer.writerow([time.monotonic()-start,name,value])
        def state(robot,event,m):
            writer.writerow([time.monotonic()-start,"state","",math.degrees(m.head_angle_rad),m.gyro.y,
                             m.left_wheel_speed_mmps,m.right_wheel_speed_mmps,m.status,int(robot.status.is_on_charger)])
        robot.events.subscribe(state,Events.robot_state)
        time.sleep(1)
        battery = robot.get_battery_state()
        assert battery.is_on_charger_platform and battery.battery_volts >= 3.5
        initial = math.degrees(robot.head_angle_rad)
        mark("request")
        grant = robot.conn.request_control(av.connection.ControlPriorityLevel.OVERRIDE_BEHAVIORS_PRIORITY,timeout=8)
        if isinstance(grant,Future):
            grant.result(timeout=10)
        mark("grant")
        def move(target):
            assert robot.status.is_on_charger
            mark("command",target)
            action = robot.behavior.set_head_angle(degrees(target))
            mark("return",action.result.code)
            assert action.result.code == 0
        try:
            time.sleep(delay)
            move(-8)
            time.sleep(.6)
            move(22)
            time.sleep(2)
        finally:
            if robot.status.is_on_charger:
                move(initial)
        time.sleep(.4)
        battery = robot.get_battery_state()
        assert battery.is_on_charger_platform
        mark("end_voltage",battery.battery_volts)


if __name__=="__main__":
    serial,ip,directory=sys.argv[1:]
    for i,delay in enumerate((1,7,7,1)):
        if i:
            time.sleep(8)
        session(serial,ip,Path(directory)/f"grant-delay-{i+1}.csv",delay)
