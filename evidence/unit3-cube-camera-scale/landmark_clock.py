"""Score SDK head/body pitch against the elevation of one distant wall feature."""
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
p=np.genfromtxt(here/"room-landmark.csv",names=True,delimiter=",")
s=np.genfromtxt(here/"dock-pitch-states.csv",names=True,delimiter=",")
cal=json.loads((here.parents[1]/"web/camera_calibration_vector2.json").read_text())
t=(s["stamp"]-s["stamp"][0])*.001
idx=p["frame"].astype(int)
# Pixel centers in a 200 x 150 area-resized view of the 800 x 600 sensor.
v=(p["y_quarter"]+.5)*4-.5
elevation=np.degrees(np.arctan((v-cal["cy_px"])/cal["fy_px"]))
enc=s["head_deg"]+s["body_pitch_deg"]
def estimate(delay):
    return np.interp(t[idx]-delay,t,enc)
train=(idx>=5)&(idx<=24)
held=(idx>=29)
def fit(delay):
    difference=estimate(delay)-elevation
    zero=np.mean(difference[train])
    return np.mean((difference[train]-zero)**2),zero
grid=np.arange(0,1.501,.005)
delay=min(grid,key=lambda x:fit(x)[0])
fig,ax=plt.subplots(figsize=(9,3.5),layout="constrained")
ax.plot(t[idx],elevation,"k.",label="Wall feature")
for d,label in [(0,"Same stamp"),(delay,"Aligned")]:
    pred=estimate(d)-fit(d)[1]
    print(label,d,*[round(np.mean((pred[m]-elevation[m])**2)**.5,2) for m in [train,held]])
    ax.plot(t[idx],pred,label=label)
ax.axvline(t[29],color=".6",ls="--",label="Held-out begins")
ax.set(xlabel="Image timestamp (s from start)",ylabel="Pitch relative to landmark (degrees)")
ax.legend()
fig.savefig(here/"landmark-clock.png",dpi=160)
