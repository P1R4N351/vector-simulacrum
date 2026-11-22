"""Read the advertising shelf cube for ten seconds without any LED commands."""
import asyncio
import time

from bleak import BleakClient,BleakScanner
from led_accel_capture import READ


async def main():
    device=await BleakScanner.find_device_by_name('Vector Cube',timeout=8)
    if device is None:
        raise RuntimeError('Cube did not advertise')
    async with BleakClient(device,timeout=22) as client:
        start=time.monotonic()
        print('sec,kind,value',flush=True)
        def receive(_,data):
            print(f'{time.monotonic()-start:.6f},packet,{data.hex()}',flush=True)
        await client.start_notify(READ,receive)
        try:
            await asyncio.sleep(10)
        finally:
            await client.stop_notify(READ)


if __name__=='__main__':
    asyncio.run(main())
