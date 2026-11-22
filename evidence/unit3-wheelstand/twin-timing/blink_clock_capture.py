"""Film the autonomous cube animation clock alongside direct BLE notifications."""
import asyncio
from pathlib import Path
import subprocess
import tempfile
import time

from bleak import BleakClient, BleakScanner
from led_accel_capture import READ, WRITE


async def main():
    folder=Path(tempfile.mkdtemp(prefix='blink-clock-'))
    print(folder,flush=True)
    device=await BleakScanner.find_device_by_name('Vector Cube',timeout=10)
    if device is None:
        raise RuntimeError('Cube unavailable')
    cams=[]
    with (folder/'receipts.csv').open('w',buffering=1) as log:
        log.write('sec,kind,value\n')
        def record(kind,value):
            log.write(f'{time.monotonic():.6f},{kind},{value}\n')
        async with BleakClient(device,timeout=22) as c:
            await c.start_notify(READ,lambda _,b:record('packet',b.hex()))
            # Static off, followed by two linked five-animation-tick holds.
            await c.write_gatt_char(WRITE,bytes([1,0,0,0,0,0,0,0,0,150,0,5,0,2,0,0,0,5,0,1]),response=True)
            try:
                for s in ('222AD609','3815D409'):
                    cams.append(subprocess.Popen(['sudo','-n','timeout','17','ffmpeg','-nostdin','-loglevel','error',
                        '-copyts','-f','v4l2','-input_format','mjpeg','-video_size','1280x720','-framerate','30','-i',
                        f'/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_{s}-video-index0',
                        '-frames:v','170','-c:v','copy','-f','nut',str(folder/(s+'.nut'))]))
                await asyncio.sleep(2)
                await c.write_gatt_char(WRITE,bytes([0,0,1,1,1,1]),response=True)
                record('start',1)
                await asyncio.sleep(8)
            finally:
                await c.write_gatt_char(WRITE,bytes(6),response=True)
                record('stop',0)
                await c.stop_notify(READ)
    if any(code!=0 for code in [p.wait() for p in cams]):
        raise RuntimeError('Incomplete capture')


if __name__=='__main__':
    asyncio.run(main())
