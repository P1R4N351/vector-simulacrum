"""Run beside the motion controller. Each file is a fresh onboard CameraFeed frame."""
import argparse
import asyncio
from pathlib import Path
import time

import anki_vector
from anki_vector.messaging import protocol


async def record(connection, directory, seconds):
    last_frame = time.monotonic()
    last_id = None
    count = 0

    async def rearm_when_idle():
        while True:
            if time.monotonic() - last_frame > 4:
                try:
                    request = protocol.EnableImageStreamingRequest(enable=True)
                    await asyncio.wait_for(connection.grpc_interface.EnableImageStreaming(request), 2)
                except asyncio.TimeoutError:
                    pass
            await asyncio.sleep(3)

    async def receive():
        nonlocal count, last_frame, last_id
        stream = connection.grpc_interface.CameraFeed(protocol.CameraFeedRequest())
        async for event in stream:
            key = (event.frame_time_stamp, event.image_id)
            if event.data and key != last_id:
                name = f"{time.time():.4f}-{key[0]}-{key[1]}.jpg"
                (directory / name).write_bytes(event.data)
                last_frame, last_id = time.monotonic(), key
                count += 1

    rearm = asyncio.create_task(rearm_when_idle())
    try:
        await asyncio.wait_for(receive(), seconds)
    except asyncio.TimeoutError:
        pass
    finally:
        rearm.cancel()
        await asyncio.gather(rearm, return_exceptions=True)
    if not count:
        raise RuntimeError("No fresh frames received")
    return count


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("serial")
    parser.add_argument("directory", type=Path)
    parser.add_argument("--ip")
    parser.add_argument("--port", type=int, default=443)
    parser.add_argument("--seconds", type=float, default=180)
    args = parser.parse_args()
    args.directory.mkdir(parents=True, exist_ok=False)
    with anki_vector.Robot(args.serial, ip=args.ip, config={"port": str(args.port)},
                           behavior_control_level=None) as robot:
        print(robot.conn.run_coroutine(record(robot.conn, args.directory, args.seconds)).result())
