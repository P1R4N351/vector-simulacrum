"""Passively sample SDK state on an occupied charger without taking control."""
import argparse
import csv
import datetime
import sys
import time

import anki_vector as av
from anki_vector.events import Events
from anki_vector.messaging import protocol

parser = argparse.ArgumentParser()
parser.add_argument("serial")
parser.add_argument("ip")
args = parser.parse_args()
writer = csv.writer(sys.stdout, lineterminator="\n")
start = time.monotonic()
writer.writerow("elapsed_s utc status head_rad lift_mm gyro_x gyro_y gyro_z accel_x accel_y accel_z prox_mm prox_quality found left_mmps right_mmps cliff_detected".split())


def receive(robot, name, m):
    p = m.prox_data
    writer.writerow([time.monotonic()-start, datetime.datetime.now(datetime.timezone.utc).isoformat(),
                     m.status, m.head_angle_rad, m.lift_height_mm, m.gyro.x, m.gyro.y, m.gyro.z,
                     m.accel.x, m.accel.y, m.accel.z, p.distance_mm, p.signal_quality, int(p.found_object),
                     m.left_wheel_speed_mmps, m.right_wheel_speed_mmps, int(bool(m.status & protocol.ROBOT_STATUS_CLIFF_DETECTED))])


with av.Robot(args.serial, ip=args.ip, behavior_control_level=None) as robot:
    for phase in ("before", "after"):
        battery = robot.get_battery_state()
        print(phase, battery.is_on_charger_platform, battery.is_charging, battery.battery_volts,
              file=sys.stderr)
        assert battery.is_on_charger_platform
        if phase == "before":
            robot.events.subscribe(receive, Events.robot_state)
            time.sleep(20)
            robot.events.unsubscribe(receive, Events.robot_state)
