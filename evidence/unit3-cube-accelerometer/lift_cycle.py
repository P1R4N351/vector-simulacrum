"""Frozen raise-to-place comparison of apparent roll at measured lift heights."""
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from decode import decode

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1]))
from simulacrum import params
values = params.values(params.load(model="vector_2"))
actions = list(csv.DictReader((here/"action-times.csv").open()))
zero = float(next(a["pod_unix_s"] for a in actions if a["action"]=="pickup"))
packets,_ = decode(here/"pickup.tsv")
t = np.array([p[0]*1e-9-zero for p in packets])
g = np.array([np.frombuffer(p[3],"<i2").reshape(3,3).mean(0)/8192 for p in packets])
robot=np.genfromtxt(here/"robot.csv",delimiter=",",names=True)
rt=robot["pod_unix_s"]-zero
base=np.median(g[(t>=-1)&(t<0)],axis=0)
# Roll is the dominant tilt. Horizontal acceleration also enters this projection.
roll=np.degrees(np.arctan2(-g[:,1],g[:,2])-np.arctan2(-base[1],base[2]))
held=np.median(roll[(t>6.5)&(t<8.3)])
raise_window=(t>=2.8)&(t<=3.85)
lower_window=(t>=10.85)&(t<=11.95)
fig,(ax,bx)=plt.subplots(1,2,figsize=(11,4.2))
scores=[]
for lag in [-.10,-.05,0,.05,.10]:
    height=np.interp(t+lag,rt,robot["lift_mm"])
    h0=np.median(robot["lift_mm"][(rt>2)&(rt<2.7)])
    h1=np.median(robot["lift_mm"][(rt>6.5)&(rt<8.3)])
    # Fit a clipped linear ramp on the raising interval only.
    grid=np.arange(np.ceil(h0),np.floor(h1)+1)
    pairs=[(lo,hi) for lo in grid for hi in grid if hi-lo>=4]
    def estimate(lo,hi):
        return held*np.clip((height-lo)/(hi-lo),0,1)
    lo,hi=min(pairs,key=lambda ab: np.mean((estimate(*ab)[raise_window]-roll[raise_window])**2))
    pred=estimate(lo,hi)
    active=lower_window&(height>35)&(height<85)
    rmse=lambda a: np.sqrt(np.mean((pred[a]-roll[a])**2))
    # A cube rolled about one horizontal axis spans L/2*(cos(theta)+sin(theta))
    # below its center. The first corner meets the shelf on the low-angle branch.
    radius=values["cube_size_mm"]/2
    center_z=height+values["cube_carry_offset_mm"][2]
    floor_roll=np.degrees(np.arcsin(np.clip(center_z/(radius*np.sqrt(2)),0,1)))-45
    supported=np.clip(floor_roll,0,held)
    floor_error=np.sqrt(np.mean((supported[lower_window]-roll[lower_window])**2))
    scores.append([lag,lo,hi,raise_window.sum(),lower_window.sum(),active.sum(),
                   rmse(raise_window),rmse(lower_window),rmse(active),floor_error])
    if lag==0:
        ax.scatter(height[raise_window],roll[raise_window],s=17,label="Raise")
        ax.scatter(height[lower_window],roll[lower_window],s=17,label="Lower")
        ax.plot([h0,lo,hi,h1],[0,0,held,held],"k--",label="Raise fit")
        ax.set(xlabel="Lift height (mm)",ylabel="Apparent cube roll from shelf (°)")
        ax.legend()
        tt=t[lower_window]
        bx.plot(tt,roll[lower_window],".-",label="Measured")
        bx.plot(tt,pred[lower_window],".--",label="Predicted from raise")
        bx.plot(tt,supported[lower_window],".-",label="Tilt limited by floor contact")
        bx.set(xlabel="Seconds from pickup command",ylabel="Apparent roll during placement (°)")
        bx.legend()
fig.tight_layout()
fig.savefig(here/"lift-cycle.png",dpi=150)
with (here/"lift-cycle.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["lift_clock_shift_s","raise_onset_mm","raise_saturation_mm","raise_packets","lower_packets",
                "lower_intermediate_packets","raise_rmse_deg","lower_rmse_deg","lower_intermediate_rmse_deg","floor_contact_lower_rmse_deg"])
    w.writerows(scores)
print(*scores,sep="\n")
