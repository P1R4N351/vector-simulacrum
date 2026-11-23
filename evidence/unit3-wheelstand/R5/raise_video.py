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
side_tiles = []
light_rows = []
# Aim a virtual camera at the shelf patch occupied by the cube; keep its pose fixed.
side_rot = cv2.Rodrigues(np.radians([24.,0,0]))[0] @ cv2.Rodrigues(np.radians([0,49.,0]))[0]
side_k = np.array([[340.,0,209.5],[0,340.,119.5],[0,0,1]])
side_maps = cv2.fisheye.initUndistortRectifyMap(k,np.zeros(4),side_rot,side_k,(420,240),cv2.CV_32FC1)
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
    side = cv2.remap(frame,*side_maps,cv2.INTER_LINEAR)
    side = cv2.bilateralFilter(side,5,18,18)
    side = cv2.addWeighted(side,1.25,cv2.GaussianBlur(side,(0,0),1),-.25,0)
    # Centroids of colored halos near the cube. Exclude the green-white backpack highlight.
    hsv = cv2.cvtColor(side,cv2.COLOR_BGR2HSV)
    hue,sat,val = cv2.split(hsv)
    yy,xx = np.indices(hue.shape)
    bounds = (xx>=40)&(xx<=225)&(yy>=35)&(yy<=210)&(sat>135)&(val>120)
    centers = []
    for color in ((hue<14)|(hue>170), (hue>=74)&(hue<=93)):
        mask = (bounds & color).astype(np.uint8)
        n, _, stats, points = cv2.connectedComponentsWithStats(mask)
        assert n > 1
        best = 1 + np.argmax(stats[1:,cv2.CC_STAT_AREA])
        centers.append(points[best])
        cv2.circle(side,tuple(np.rint(points[best]).astype(int)),8,(245,245,245),1,cv2.LINE_AA)
    red, cyan = centers
    light_rows.append([idx, time, lift, *red, *cyan, np.linalg.norm(red-cyan)])
    side = cv2.copyMakeBorder(side,0,24,0,0,cv2.BORDER_CONSTANT)
    cv2.putText(side,f"{time:.2f} s    wrist ~{lift:.0f} mm",(8,257),0,.52,(255,255,255),1,cv2.LINE_AA)
    side_tiles.append(side)
cv2.imwrite(str(here/"raise-video.jpg"),np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,8,2)]))
cv2.imwrite(str(here/"raise-side.jpg"),np.vstack([np.hstack(side_tiles[i:i+2]) for i in range(0,8,2)]))
with (here/"raise-lights.csv").open("w") as stream:
    w=csv.writer(stream)
    w.writerow("frame receipt_s wrist_mm red_x_px red_y_px cyan_x_px cyan_y_px separation_px".split())
    w.writerows(light_rows)
