"""Register the archived video to its four timestamped rectified stills."""
import csv
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np

here = Path(__file__).resolve().parent
panel = cv2.imread(str(here/"rectified-carry.jpg"))
anchors = [24.99,29.99,34.98,47.01]
# Match on the moving robot and cube; stationary background would dominate a full-frame score.
tiles = [panel[y+175:y+350,x:x+340] for y in [0,408] for x in [0,640]]
f = 320/np.tan(np.radians(65))
k = np.array([[249.2,0,480],[0,249.2,270],[0,0,1]])
virtual = np.array([[f,0,319.5],[0,f,179.5],[0,0,1]])
rot = cv2.Rodrigues(np.radians([10.,0,0]))[0]
maps = cv2.fisheye.initUndistortRectifyMap(k,np.zeros(4),rot,virtual,(640,360),cv2.CV_32FC1)
video = cv2.VideoCapture(str(here/"carry.mp4"))
probe = subprocess.check_output(["ffprobe","-v","error","-select_streams","v:0","-show_entries",
                                 "frame=best_effort_timestamp_time","-of","json",str(here/"carry.mp4")])
pts = np.array([float(r["best_effort_timestamp_time"]) for r in json.loads(probe)["frames"]])
errors = []
while True:
    ok, frame = video.read()
    if not ok:
        break
    crop = cv2.remap(frame,*maps,cv2.INTER_LINEAR)[175:350,:340]
    errors.append([cv2.norm(crop,tile,cv2.NORM_L1)/crop.size for tile in tiles])
matches = np.argmin(errors,axis=0)
assert len(pts) == len(errors)
offsets = anchors - pts[matches]
print(matches,offsets)
with (here/"video-registration.csv").open("w") as stream:
    w=csv.writer(stream)
    w.writerow(["reference_s","best_frame","video_s","offset_s","mean_abs_pixel_difference"])
    for i,frame in enumerate(matches):
        w.writerow([anchors[i],frame,pts[frame],offsets[i],errors[frame][i]])
state = np.genfromtxt(here/"state.csv.gz",delimiter=",",names=True)
# Show the rise around the first 1 mm crossing, using the nearby reference as the clock anchor.
tiles = []
targets = {int(np.argmin(abs(pts + offsets[0] - sec))) for sec in
           [25.90,26.08,26.19,26.33,26.48,26.65,26.80,27.02]}
video.release()
video = cv2.VideoCapture(str(here/"carry.mp4"))
for idx in range(max(targets)+1):
    ok, frame = video.read()
    assert ok
    if idx not in targets:
        continue
    view = cv2.remap(frame,*maps,cv2.INTER_LINEAR)[175:355,:335]
    view = cv2.bilateralFilter(view,5,18,18)
    view = cv2.addWeighted(view,1.25,cv2.GaussianBlur(view,(0,0),1),-.25,0)
    view = cv2.copyMakeBorder(view,0,25,0,0,cv2.BORDER_CONSTANT)
    time = pts[idx]+offsets[0]
    lift = np.interp(time,state["sec"],state["lift"])
    cv2.putText(view,f"{time:.2f} s    wrist ~{lift:.0f} mm",(8,199),0,.52,(255,255,255),1,cv2.LINE_AA)
    tiles.append(view)
cv2.imwrite(str(here/"raise-video.jpg"),np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,8,2)]))
