"""Bracket changes in apparent cube tilt, wrist height and simulated support in receipt time."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))

def packets(data, keys, time_key):
    groups=data.reshape(-1,3)
    assert np.all(groups["slot"]==[0,1,2]) and np.all(groups[time_key]==groups[time_key][:,:1])
    return groups[time_key][:,0], np.stack([groups[k].mean(axis=1) for k in keys],axis=1)/8192

def angles(g,reference):
    u=reference/np.linalg.norm(reference)
    return np.degrees(np.arctan2(np.linalg.norm(np.cross(g,u),axis=1),g@u))

def bracket(t,flag):
    # Two successive receipts within 100 ms confirm persistence. Report the first.
    pairs=np.flatnonzero(flag[1:] & flag[:-1] & (np.diff(t)<=.1))
    if not len(pairs): return np.nan,np.nan
    i=pairs[0]
    return t[i-1] if i else np.nan,t[i]

def controls(name):
    root=here.parent/"unit3-wheelstand"/name
    t,g=packets(np.genfromtxt(root/"cube.csv.gz",delimiter=",",names=True),"xyz","sec")
    events=read(root/"actions.csv")
    for move in read(root/"moves.csv"):
        if move["timing_valid"]!="1": continue
        label=" ".join(move[k] for k in ("condition","direction","repeat"))
        start,end=[float(next(e["sec"] for e in events if e["action"]==label and e["kind"]==k)) for k in ("begin","end")]
        a=angles(g,np.median(g[(t>=start-.8)&(t<start-.1)],axis=0))
        w=(t>=start)&(t<=end)
        yield t[w]-start,a[w]

training=list(controls("D1"))
threshold=np.quantile(np.concatenate([a for _,a in training]),.99)
test=list(controls("D2"))
false=sum(np.isfinite(bracket(t,a>threshold)[1]) for t,a in test)
print("D1 strokes",len(training),"threshold_deg",threshold,"D2 alarms",false,"/",len(test))
events=read(here/"action-times.csv")
start={a:float(next(e["pod_unix_s"] for e in events if e["event"]=="begin" and e["action"]==a)) for a in ("pickup","place")}
table=np.genfromtxt(here/"pickup.csv",names=True,delimiter=",",dtype=None,encoding="utf8")
t,g=packets(table,["x_counts","y_counts","z_counts"],"pod_rx_unix_ns")
t=t*1e-9-start["pickup"]
robot=np.genfromtxt(here/"robot.csv",names=True,delimiter=",")
rt=robot["pod_unix_s"]-start["pickup"]
sim=read(here/"free-pickup.csv")
place=start["place"]-start["pickup"]
fig,axes=plt.subplots(2,2,figsize=(11,6),layout="constrained",sharex="col")
with (here/"motion-brackets.csv").open("w") as f:
    writer=csv.writer(f,lineterminator="\n")
    writer.writerow(["action","signal","preceding_receipt_s","first_crossing_receipt_s"])
    for c,(name,offset,reference) in enumerate([("pickup",0,(-.7,0)),("place",place,(place-.7,place))]):
        duration=4.2 if c==0 else 2.8
        w=(t>=offset)&(t<=offset+duration)
        rel=t[w]-offset
        a=angles(g[w],np.median(g[(t>=reference[0])&(t<reference[1])],axis=0))
        # Both primary references end at the command under test.
        rest=(rt>=offset-.25)&(rt<offset)
        obs=(rt>=offset)&(rt<=offset+duration)
        h=robot["lift_mm"][obs]
        def mark(signal,times,flags):
            lo,hi=bracket(times,flags)
            writer.writerow([name,signal,lo,hi])
            return hi
        first=mark("cube apparent tilt",rel,a>threshold)
        mark("wrist height >2mm",rt[obs]-offset,abs(h-np.median(robot["lift_mm"][rest]))>2)
        if c:
            carried=(t>=6.5)&(t<=8.3)
            angle_from_carry=angles(g[w],np.median(g[carried],axis=0))
            mark("cube apparent tilt vs pre-turn carry",rel,angle_from_carry>threshold)
        ax,bx=axes[:,c]
        ax.plot(rel,a,"k.-",label="Cube apparent tilt")
        ax.axhline(threshold,ls=":",color="grey",label="Empty-fork threshold")
        if np.isfinite(first): ax.axvline(first,ls=":",color="black")
        bx.plot(rt[obs]-offset,h,"k.-",label="Physical wrist")
        for label in dict.fromkeys(row["initialization"] for row in sim):
            rows=[r for r in sim if r["initialization"]==label]
            st=np.array([float(r["time_s"]) for r in rows])
            z=np.array([float(r["center_z_mm"]) for r in rows])
            sel=(st>=offset)&(st<=offset+duration)
            initial=np.median(z[(st>=offset-.2)&(st<offset)])
            mark(label+" center >2mm",st[sel]-offset,abs(z[sel]-initial)>2)
            bx.plot(st[sel]-offset,z[sel],lw=1,label=label)
            if c==0:
                displacement=np.hypot([float(r["cube_dx_mm"]) for r in rows],[float(r["cube_dy_mm"]) for r in rows])
                mark(label+" planar >2mm",st[sel],displacement[sel]>2)
        if c==0:
            camera=np.genfromtxt(here/"shelf-slide.csv",names=True,delimiter=",")
            first_moved=np.flatnonzero(camera["estimated_mm"]>2)[0]
            writer.writerow([name,"overhead planar >2mm",*camera["time_s"][first_moved-1:first_moved+1]])
            ax.axvspan(*camera["time_s"][first_moved-1:first_moved+1],alpha=.13,color="teal",label="Camera slide bracket")
        if c: ax.legend(fontsize=7)
        ax.set(title=name.capitalize(),ylabel="Angle from reference (°)")
        bx.set(xlabel="Seconds after command",ylabel="Height (mm)")
axes[0,0].legend(fontsize=8)
axes[1,0].legend(fontsize=7)
fig.savefig(here/"motion-brackets.png",dpi=150)
