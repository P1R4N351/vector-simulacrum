"""Film the autonomous cube animation clock alongside direct BLE notifications."""
import asyncio
from pathlib import Path
import subprocess
import tempfile
import time

from bleak import BleakClient, BleakScanner
READ='43ef14af-5fb1-7b81-3647-2a9477824cab'


async def main():
    folder=Path(tempfile.mkdtemp(prefix='cube-rock-'))
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
            try:
                for s in ('222AD609','3815D409'):
                    cams.append(subprocess.Popen(['sudo','-n','timeout','-s','INT','92','ffmpeg','-nostdin','-loglevel','error',
                        '-copyts','-f','v4l2','-input_format','mjpeg','-video_size','1280x720','-framerate','30','-i',
                        f'/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_{s}-video-index0',
                        '-frames:v','2100','-c:v','copy','-f','nut',str(folder/(s+'.nut'))]))
                await asyncio.sleep(2)
                if any(p.poll() is not None for p in cams):
                    raise RuntimeError('Camera exited during warmup')
                record('start',1)
                print('Recording',flush=True)
                await asyncio.sleep(85)
            finally:
                record('stop',0)
                await c.stop_notify(READ)
    if any(code not in (0,124) for code in [p.wait() for p in cams]):
        raise RuntimeError('Incomplete capture')


if __name__=='__main__':
    asyncio.run(main())
