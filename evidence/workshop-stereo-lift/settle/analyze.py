"""Use the previous stereo session for image templates and score this independent repeat."""
import csv
from pathlib import Path
import cv2
import numpy as np
import matplotlib.pyplot as plt

def rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))

def load(folder):
    sdk=rows(folder/"sdk.csv")
    states=[r for r in sdk if r["event"]=="state"]
    t=np.array([float(r["relative_s"]) for r in states])
    h=np.array([float(r["lift_mm"]) for r in states])
    camera=np.genfromtxt(folder/"camera.csv",names=True,delimiter=",")["relative_s"]
    cap=cv2.VideoCapture(str(folder/"lift.mp4"))
    images=[]
    while True:
        ok,frame=cap.read()
        if not ok:break
        images.append(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY))
    cap.release()
    assert len(camera)==len(images)
    return sdk,states,t,h,camera,np.array(images)

source,_,st,sh,ct,images=load(Path(".."))
split=float([r for r in source if r["event"]=="command"][4]["relative_s"])
train=(ct>=0)&(ct<split)
sdk,states,t,h,camera,frames=load(Path("."))
pred=[]
for x in [205,295]:
    bank=images[train,12:133,x-11:x+12].astype("float32")
    values=np.interp(ct[train],st,sh)
    pred.append(np.array([values[np.argmin(np.mean(abs(bank-im[12:133,x-11:x+12]),axis=(1,2)))] for im in frames]))
obs=np.interp(camera,t,h)
moving=abs(np.gradient(obs,camera))>5
print("Optical all/moving MAE:",[(np.mean(abs(p-obs)),np.mean(abs((p-obs)[moving]))) for p in pred],len(camera),moving.sum())
cmds=[r for r in sdk if r["event"]=="command"]
dones=[float(r["relative_s"]) for r in sdk if r["event"]=="done"]
flag=np.array([int(r["in_position"]) for r in states])
fig,(a,b)=plt.subplots(2,1,figsize=(10,6),layout="constrained")
a.plot(t,h,color="black",label="Encoder")
for label,p in zip(["Left ZED","Right ZED"],pred):a.scatter(camera,p,s=5,label=label)
a.set(ylabel="Height (mm)",xlabel="Seconds after first command")
a.legend(ncol=3)
with open("moves.csv","w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["move","target","last_unsettled_s","first_settled_s","return_s","return_minus_first_settled_s"])
    for i,(r,end) in enumerate(zip(cmds,dones)):
        start=float(r["relative_s"])
        stop=float(cmds[i+1]["relative_s"]) if i+1<len(cmds) else t[-1]
        off=np.flatnonzero((t>=start)&(t<stop)&(flag==0))
        assert len(off)
        j=off[0]
        k=next(k for k in range(j+1,len(t)) if flag[k])
        assert t[k]<stop
        w.writerow([i+1,r["target"],t[k-1]-start,t[k]-start,end-start,end-t[k]])
        b.plot([i+1,i+1],[t[k-1]-start,t[k]-start],color="tab:blue",lw=6)
        b.scatter(i+1,end-start,color="tab:orange")
b.plot([],[],color="tab:blue",lw=6,label="In-position transition bracket")
b.scatter([],[],color="tab:orange",label="SDK return")
b.set(xticks=range(1,9),xlabel="Sequential move",ylabel="Seconds after command");b.legend()
fig.savefig("settling.png",dpi=150)
