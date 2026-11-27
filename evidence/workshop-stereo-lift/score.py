"""Estimate encoder-equivalent height from each eye using only the first four moves."""
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np

with open("sdk.csv") as f:
    rows=list(csv.DictReader(f))
states=[r for r in rows if r["event"]=="state"]
commands=[r for r in rows if r["event"]=="command"]
t=np.genfromtxt("camera.csv",delimiter=",",names=True)["relative_s"]
st=np.array([float(r["relative_s"]) for r in states])
height=np.array([float(r["lift_mm"]) for r in states])
observed=np.interp(t,st,height)
split=float(commands[4]["relative_s"])
train=np.flatnonzero((t>=0)&(t<split))
test=t>=split
cap=cv2.VideoCapture("lift.mp4")
frames=[]
while True:
    ok,frame=cap.read()
    if not ok:break
    frames.append(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY))
cap.release()
assert len(frames)==len(t)
fig,ax=plt.subplots(figsize=(10,4),layout="constrained")
ax.plot(st,height,color="black",label="Lift encoder")
predictions=[]
for label,x in [("Left eye",205),("Right eye",295)]:
    # Narrow strips contain the outer lift post through its travel, away from the display.
    features=np.float32([im[12:133,x-11:x+12] for im in frames])
    predictions.append(np.array([observed[train[np.argmin(np.mean(abs(features[train]-feature),axis=(1,2)))]] for feature in features]))
    error=predictions[-1][test]-observed[test]
    print(label,"held-out frames",test.sum(),"MAE mm",np.mean(abs(error)),"p90 mm",np.quantile(abs(error),.9))
    ax.plot(t[test],predictions[-1][test],".",ms=3,label=label)
ax.axvline(split,color="gray",ls="--")
ax.set(xlim=(-.4,t[-1]),xlabel="Seconds from first lift command",ylabel="Encoder-equivalent height (mm)",title="Docked workshop Vector · repeated 52 and 64 mm raises")
ax.legend()
fig.savefig("lift-overlay.png",dpi=160)
np.savetxt("eye-estimates.csv",np.c_[t,observed,*predictions,test],delimiter=",",header="t,encoder_mm,left_mm,right_mm,held",comments="")
