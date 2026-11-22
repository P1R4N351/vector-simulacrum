"""Record camera packet metadata and nearby SDK gyro samples without behavior control."""
import asyncio
import csv
import sys
import threading
import time
import anki_vector
from anki_vector.events import Events
from anki_vector.messaging import protocol

rows = []
lock = threading.Lock()
def record(kind,stamp,frame="",y="",z=""):
    with lock:
        rows.append([time.monotonic_ns(),kind,stamp,frame,y,z])
def state(robot,name,msg):
    record("state",msg.last_image_time_stamp,y=msg.gyro.y,z=msg.gyro.z)
def observed(robot,name,msg):
    record("object",msg.timestamp)
async def camera(robot):
    feed=robot.conn.grpc_interface.CameraFeed(protocol.CameraFeedRequest())
    try:
        async for frame in feed:
            record("camera",frame.frame_time_stamp,frame.image_id)
    finally:
        feed.cancel()
with anki_vector.Robot("0a1b2c3d",ip="192.0.2.10",behavior_control_level=None) as robot:
    robot.events.subscribe(state,Events.robot_state)
    robot.events.subscribe(observed,Events.robot_observed_object)
    task=asyncio.run_coroutine_threadsafe(camera(robot),robot.conn.loop)
    try:
        task.result(timeout=12)
    except TimeoutError:
        task.cancel()
    if not any(r[1]=="camera" for r in rows):
        raise RuntimeError("no camera packets")
writer=csv.writer(sys.stdout,lineterminator="\n")
writer.writerow(["receipt_s","kind","stamp","frame_id","gyro_y","gyro_z"])
for ns,*rest in sorted(rows):
    writer.writerow([(ns-min(r[0] for r in rows))*1e-9,*rest])
