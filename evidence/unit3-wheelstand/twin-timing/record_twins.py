"""Record both shelf cameras around a physical action, preserving V4L2 timestamps."""
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

folder=Path(sys.argv[1])
folder.mkdir(exist_ok=False)
if shutil.disk_usage(folder).free < 2_000_000_000:
    raise RuntimeError('Insufficient recording space')
active=[]
try:
    for s in ('222AD609','3815D409'):
        dev=f'/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_{s}-video-index0'
        active.append(subprocess.Popen(['ffmpeg','-loglevel','error','-nostdin','-copyts','-f','v4l2','-input_format','mjpeg',
                                        '-video_size','1280x720','-framerate','30','-i',dev,'-c:v','copy','-f','nut',str(folder/(s+'.nut'))]))
    start=time.monotonic()
    with (folder/'clocks.csv').open('w',buffering=1) as f:
        f.write('monotonic_s,unix_s\n')
        while time.monotonic()-start<150 and not (folder/'stop').exists():
            if any(p.poll() is not None for p in active):raise RuntimeError('camera stopped')
            f.write(f'{time.monotonic():.6f},{time.time():.6f}\n')
            time.sleep(.3)
finally:
    for p in active:
        if p.poll() is None:p.send_signal(signal.SIGINT)
    for p in active:p.wait(timeout=10)
