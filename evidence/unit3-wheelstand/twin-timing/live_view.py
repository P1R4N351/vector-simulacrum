"""Capture a corrected shelf preview from each twin over the capture host's SSH alias."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import shlex
import subprocess
import sys

import cv2
import numpy as np
from shelf_view import SERIALS, ShelfView


def capture(serial):
    device = f'/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_{serial}-video-index0'
    command = ['sudo','-n','timeout','10','ffmpeg','-nostdin','-loglevel','error','-f','v4l2',
               '-input_format','mjpeg','-video_size','1280x720','-i',device,'-ss','0.6',
               '-frames:v','1','-vcodec','mjpeg','-f','image2pipe','-']
    encoded = subprocess.check_output(['ssh','-o','ConnectTimeout=5',sys.argv[1],shlex.join(command)],timeout=20)
    frame = cv2.imdecode(np.frombuffer(encoded,np.uint8),cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError(f'No frame from {serial}')
    panel = ShelfView(serial,frame).apply(frame)
    return panel


if __name__=='__main__':
    with ThreadPoolExecutor(2) as pool:
        panels = list(pool.map(capture,SERIALS))
    if '--mirror' in sys.argv[3:]:
        from mirror_view import comparison
        preview = comparison(*panels)
    else:
        preview = np.hstack([cv2.resize(panel,(600,264),interpolation=cv2.INTER_AREA) for panel in panels])
    if not cv2.imwrite(str(Path(sys.argv[2])),preview):
        raise OSError('Could not save shelf preview')
