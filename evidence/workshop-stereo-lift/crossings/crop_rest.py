"""Rectify fresh workshop stereo pairs into the same robot-only crop as the lift trials."""
import csv
from pathlib import Path
import cv2
import numpy as np
from zed2_nomad.zed2_calib import build_rectifier, rectify_pair

rect = build_rectifier(next((Path.home()/'zed2-nomad/calib').glob('*.conf')))
with open('lift/frames.csv') as source, open('rest-camera.csv','w') as out:
    writer = csv.writer(out,lineterminator='\n')
    writer.writerow(('index','relative_s'))
    clip = cv2.VideoWriter('rest.avi',cv2.VideoWriter_fourcc(*'MJPG'),15,(500,232))
    for i, row in enumerate(csv.DictReader(source)):
        t = float(row['receipt_epoch_s'])
        if i == 0:
            start = t
        frame = cv2.imread(f"lift/{int(row['frame']):04d}.jpg")
        a,b = rectify_pair(rect,frame[:,:1280],frame[:,1280:])
        clip.write(np.hstack((a[170:402,682:932],b[170:402,378:628])))
        writer.writerow((row['frame'],t-start))
    clip.release()
