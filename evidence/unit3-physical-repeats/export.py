"""Export paired physical data; usage: export.py RAW_DIRECTORY RUN INPUT_STEM."""
import csv
import gzip
import json
import subprocess
import sys
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
raw,name,stem=Path(sys.argv[1]),sys.argv[2],sys.argv[3]
out=here/name
out.mkdir(exist_ok=True)


def output(name, columns, rows):
    path=out/name
    opener=gzip.open if path.suffix==".gz" else open
    with opener(path,"wt") as f:
        w=csv.writer(f,lineterminator="\n");w.writerow(columns);w.writerows(rows)


events=[]
decoder=json.JSONDecoder()
for line in (raw/(stem+".jsonl")).read_text().splitlines():
    # SDK stdout can place two JSON objects on the same line.
    while line:
        try:e,off=decoder.raw_decode(line)
        except ValueError:break
        if isinstance(e,dict) and "kind" in e:events.append(e)
        line=line[off:].lstrip()
start=next(e["t"] for e in events if e["kind"]=="begin" and e["action"]=="undock")
robot=[];actions=[];objects=[];ranges=[];views=[]
for e in events:
    t=round(e["t"]-start,5);kind=e["kind"];p=e.get("data",{})
    if kind=="robot_state":
        pose=p.get("pose",{});gyro=p.get("gyro",{});accel=p.get("accel",{})
        prox=p.get("prox_data",{})
        ranges.append([t,prox.get("distance_mm",""),prox.get("signal_quality",0),prox.get("found_object",False),prox.get("unobstructed",False)])
        robot.append([t,pose.get("origin_id",0),pose.get("x",0),pose.get("y",0),p.get("pose_angle_rad",0),
                      p.get("head_angle_rad",0),p.get("lift_height_mm",0),p.get("left_wheel_speed_mmps",0),
                      p.get("right_wheel_speed_mmps",0),p.get("carrying_object_id",0),p.get("status",0),
                      *[gyro.get(k,0) for k in "xyz"],*[accel.get(k,0) for k in "xyz"]])
    elif kind=="view":
        views.append([t, e["fresh_cube_age"] if e["fresh_cube_age"]<300 else "", e["accel_age"] if e["accel_age"]<300 else ""])
    elif kind in ("begin","end","trial","carry","battery"):
        actions.append([t,kind,e.get("action",""),e.get("number",""),e.get("reported","")])
    elif kind.startswith("object_") or (kind=="robot_observed_object" and p.get("object_family")=="LIGHT_CUBE"):
        pose=p.get("pose",{});objects.append([t,kind,p.get("object_id",""),pose.get("origin_id",""),pose.get("x",""),pose.get("y","")])
output("robot.csv.gz",["receipt_s","origin","x_mm","y_mm","yaw_rad","head_rad","lift_mm","left_mmps","right_mmps","carry_id","status","gx","gy","gz","ax","ay","az"],robot)
output("range.csv.gz",["receipt_s","distance_mm","signal_quality","found","unobstructed"],ranges)
if views:output("views.csv",["receipt_s","since_last_cube_detection_s","since_last_accel_packet_s"],views)
output("actions.csv",["receipt_s","event","action","trial","reported_carry"],actions)
output("object-events.csv",["receipt_s","event","object_id","origin","x_mm","y_mm"],objects)
xyz=[];radio=[]
for line in (raw/(stem+".tsv")).read_text().splitlines():
    host,uptime,payload=line.split();t=round(int(host)*1e-9-start,6);data=bytes.fromhex(payload)
    radio.append([t,uptime,payload])
    if len(data)==20 and data[0]==2:
        xyz.extend([[t,i,data[1],*v] for i,v in enumerate(np.frombuffer(data,"<i2",offset=2).reshape(3,3))])
output("radio.csv.gz",["receipt_s","robot_uptime_s","payload_hex"],radio)
output("cube.csv.gz",["receipt_s","slot","tap_count","x_counts","y_counts","z_counts"],xyz)
arr=np.array(xyz);rob=np.array(robot)
fig,ax=plt.subplots(2,1,sharex=True,figsize=(10,5),layout="constrained")
if len(arr):
    ax[0].plot(arr[:,0],arr[:,3:]/8192,lw=.65,label=list("XYZ"))
    ax[0].legend(ncol=3)
ax[0].set_ylabel("Cube specific force (g)")
ax[1].plot(rob[:,0],rob[:,6],color="#7040a4");ax[1].set(ylabel="Robot lift (mm)",xlabel="Seconds from undock imperative")
for t,k,act,*_ in actions:
    if k=="begin" and act in ("pickup","place","dock"):
        ax[0].axvline(t,ls=":",color=".5")
        ax[0].text(t,.96,act,transform=ax[0].get_xaxis_transform(),fontsize=8,va="top")
fig.savefig(out/"traces.png",dpi=140);plt.close(fig)
frames=sorted((raw/"frames").glob("*.jpg"));times=np.array([float(f.stem)-start for f in frames])
rate=5
cmd=["ffmpeg","-y","-loglevel","error","-f","rawvideo","-pix_fmt","bgr24","-s","640x400","-r",str(rate),"-i","-","-c:v","libx264","-crf","24","-pix_fmt","yuv420p","-movflags","+faststart",str(out/"overhead.mp4")]
with subprocess.Popen(cmd,stdin=subprocess.PIPE) as enc:
    for t in np.arange(0,min(times[-1],rob[-1,0])+1/rate,1/rate):
        i=np.searchsorted(times,t).clip(1,len(times)-1);i-=abs(times[i-1]-t)<abs(times[i]-t)
        im=cv2.resize(cv2.imread(str(frames[i])),(640,400))
        if abs(times[i]-t)>.55:im[:]=0
        cv2.putText(im,f"{name}   {t:.1f} s",(10,25),0,.65,(255,255,255),2)
        enc.stdin.write(im.tobytes())
    enc.stdin.close()
    if enc.wait():raise RuntimeError("Video encoder failed")
print(name,len(radio),len(xyz),len(robot),len(objects),actions[-4:])
