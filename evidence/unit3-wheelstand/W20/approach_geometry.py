"""Estimate cube location relative to chassis at the final raised-fork report."""
import csv
import json
import math
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
runs = ("W4","W8","W12","W20")


def extract(paths):
    out = []
    decoder = json.JSONDecoder()
    for run,path in zip(runs,paths,strict=True):
        events = []
        for line in Path(path).read_text().splitlines():
            line=line.strip()
            while line:
                event,end=decoder.raw_decode(line)
                events.append(event)
                line=line[end:].strip()
        events.sort(key=lambda e:e["t"])
        zero=next(e["t"] for e in events if e.get("action")=="pop_a_wheelie" and e["kind"]=="begin")
        states=[e for e in events if e["kind"]=="robot_state" and e["t"]>=zero]
        before=next(a for a,b in zip(states,states[1:]) if a["data"]["lift_height_mm"]>80 and
                    b["data"]["lift_height_mm"]<a["data"]["lift_height_mm"]-3)
        robot=before["data"]["pose"]
        sightings=[e for e in events if e["kind"]=="robot_observed_object" and e["t"]<before["t"] and
                   e["data"].get("object_family")=="LIGHT_CUBE" and
                   e["data"]["pose"].get("origin_id",0)==robot.get("origin_id",0)]
        for e in sightings[-3:]:
            pose=e["data"]["pose"]
            w,x,y,z=[pose.get("q"+str(i),0) for i in range(4)]
            yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            out.append([run,before["t"]-zero,e["t"]-zero,robot["x"],robot["y"],
                        before["data"]["pose_angle_rad"],pose["x"],pose["y"],yaw])
    with (here/"approach-input.csv").open("w") as f:
        wr=csv.writer(f,lineterminator="\n")
        wr.writerow("run raised_s sighting_s robot_x_mm robot_y_mm robot_yaw cube_x_mm cube_y_mm cube_yaw".split())
        wr.writerows(out)


if len(sys.argv)>1:
    extract(sys.argv[1:])
fig,axes=plt.subplots(1,2,figsize=(9,4),layout="constrained")
with (here/"approach-input.csv").open() as f, (here/"approach-geometry.csv").open("w") as out:
    wr=csv.writer(out,lineterminator="\n")
    wr.writerow(["run","sighting_age_s","forward_mm","left_mm","nearest_face_skew_deg"])
    for r in csv.DictReader(f):
        run=r.pop("run")
        d={k:float(v) for k,v in r.items()}
        yaw=d["robot_yaw"]
        dx,dy=d["cube_x_mm"]-d["robot_x_mm"], d["cube_y_mm"]-d["robot_y_mm"]
        fw=dx*np.cos(yaw)+dy*np.sin(yaw)
        left=-dx*np.sin(yaw)+dy*np.cos(yaw)
        skew=(np.rad2deg(d["cube_yaw"]-yaw)+45)%90-45
        age=d["raised_s"]-d["sighting_s"]
        wr.writerow([run,age,fw,left,skew])
        axes[0].plot(fw,left,"o",color=f"C{runs.index(run)}",label=run)
        axes[1].plot(age,skew,"o",color=f"C{runs.index(run)}")
handles,labels=axes[0].get_legend_handles_labels()
keys=dict(zip(labels,handles))
axes[0].legend(keys.values(),keys.keys())
axes[0].axhline(0,color=".6",lw=.7)
axes[0].set(xlabel="Cube center forward of chassis origin (mm)",ylabel="Cube center to left (mm)",aspect="equal")
axes[1].set(xlabel="Age of cube sighting at raised-fork sample (s)",ylabel="Signed skew to nearest face (°)")
fig.savefig(here/"approach-geometry.png",dpi=160)

# Check the cube's motion independently around the final two W20 pose updates.
import cv2
video=cv2.VideoCapture(str(here/"dock-request.mp4"))
frames={}
for frame in range(62):
    ok,img=video.read()
    assert ok
    if frame>=52: frames[frame]=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
with (here/"approach-video.csv").open("w") as f:
    wr=csv.writer(f,lineterminator="\n")
    wr.writerow(["frame","seconds_after_request","view","shift_x_px","shift_y_px","correlation"])
    for view,x,y in [("left",383,144),("right",584,111)]:
        side=30
        patch=frames[56][y:y+side,x:x+side]
        for frame,img in frames.items():
            region=img[y-12:y+side+12,x-12:x+side+12]
            _,score,_,pos=cv2.minMaxLoc(cv2.matchTemplate(region,patch,cv2.TM_CCOEFF_NORMED))
            wr.writerow([frame,frame/8-3.097,view,pos[0]-12,pos[1]-12,score])
