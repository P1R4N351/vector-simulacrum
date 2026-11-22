"""Short direct BLE light/sensor measurement on the stationary shelf cube."""
import asyncio
import time

from bleak import BleakClient, BleakScanner

READ = '43ef14af-5fb1-7b81-3647-2a9477824cab'
WRITE = '0ea75290-6759-a58d-7948-598c4e02d94a'


async def main():
    device = await BleakScanner.find_device_by_name('Vector Cube',timeout=8)
    if device is None:
        raise RuntimeError('Cube did not advertise')
    async with BleakClient(device,timeout=22) as client:
        start = time.monotonic()
        def record(kind,value):
            print(f'{time.monotonic()-start:.6f},{kind},{value}',flush=True)
        print('sec,kind,value',flush=True)
        await client.start_notify(READ, lambda _,b: record('packet',b.hex()))
        try:
            # Frame 0 is dark. Frame 1 is green and links back to dark.
            frames = bytes([1,0,0,0,0,100,0,0,0,220,0,100,0,0,0,0,0,100,0,0])
            await client.write_gatt_char(WRITE,frames,response=True)
            for idx in [0,1,0,1,0,1,0,1,0]:
                await client.write_gatt_char(WRITE,bytes([0,0]+[idx]*4),response=True)
                record('light',idx)
                await asyncio.sleep(.8)
        finally:
            await client.write_gatt_char(WRITE,bytes(6),response=True)
            await client.stop_notify(READ)


if __name__=='__main__':
    asyncio.run(main())
