"""Score quiet gravity windows in later shelf captures under frozen observation gains."""
import csv
import gzip
import json
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
EVIDENCE = HERE.parent


def sessions():
    wheelstand = EVIDENCE/"unit3-wheelstand"
    for path in sorted(wheelstand.rglob("cube.csv.gz")):
        if path.parent.name not in ("W1","W2","W3","W4"):
            yield str(path.parent.relative_to(wheelstand)), path
    yield "R8/rest", wheelstand/"R8/settled.csv.gz"
    yield "spin", EVIDENCE/"unit3-spin-cube/cube.csv.gz"


def windows(path):
    by_trial = defaultdict(list)
    with gzip.open(path,"rt") as stream:
        for row in csv.DictReader(stream):
            by_trial[row.get("trial","")].append([float(row.get("packet_s",row.get("sec"))),
                *[float(row[key])/8192 for key in "xyz"]])
    for trial, samples in by_trial.items():
        data = np.asarray(samples)
        t = data[:,0] - data[0,0]
        for sec in range(int(t[-1])):
            use = (t>=sec)&(t<sec+1)
            if use.sum()<45 or np.ptp(t[use])<.75:
                continue
            v = data[use,1:]
            mean=v.mean(axis=0)
            norm=np.linalg.norm(mean)
            scatter=np.sqrt(np.mean(np.sum((v-mean)**2,axis=1)))
            if scatter>.012 or not .9<norm<1.1:
                continue
            axis=np.argmax(abs(mean))
            tilt=np.degrees(np.arccos(abs(mean[axis])/norm))
            face=("+" if mean[axis]>0 else "-")+"XYZ"[axis] if tilt<=10 else "tilted"
            yield trial,sec,face,mean,scatter


def main():
    with (EVIDENCE.parent/"web/src/sim/shelf_cube.json").open() as stream:
        model=json.load(stream)
    scale=np.array(model["cube_accel_scale"])
    rows=[]
    for label,path in sessions():
        for trial,sec,face,mean,scatter in windows(path):
            errors=[1000*(np.linalg.norm(mean/gain)-1) for gain in (np.ones(3),scale,np.repeat(scale[2],3))]
            rows.append([label,trial,sec,face,*mean,scatter,*errors])
    with (HERE/"face-coverage.csv").open("w") as f:
        out=csv.writer(f,lineterminator="\n")
        out.writerow("session trial second face mean_x_g mean_y_g mean_z_g rms_g nominal_mg canonical_mg common_mg".split())
        out.writerows(rows)
    # Fit the remaining axis on the pre-reverse carry only. Keep Y/Z frozen.
    carry=[r for r in rows if r[0]=="R5" and r[3]=="tilted"]
    pre=np.mean([r[4:7] for r in carry if r[2]<35],axis=0)
    post=np.mean([r[4:7] for r in carry if r[2]>=35],axis=0)
    x_gain=abs(pre[0])/np.sqrt(1-np.sum((pre[1:]/scale[1:])**2))
    with (HERE/"action-times.csv").open() as stream:
        cue=next(float(r["pod_unix_s"]) for r in csv.DictReader(stream) if r["action"]=="pickup" and r["event"]=="begin")
    with (HERE/"pickup.csv").open() as stream:
        earlier=np.array([[float(r[k+"_counts"])/8192 for k in "xyz"] for r in csv.DictReader(stream)
                         if 6.5<=int(r["pod_rx_unix_ns"])*1e-9-cue<8.3])
    with (HERE/"face-x-transfer.csv").open("w") as stream:
        out=csv.writer(stream,lineterminator="\n")
        out.writerow("window x_gain nominal_x_error_mg fitted_x_error_mg".split())
        for name,m in (("R5 fit",pre),("R5 post-reverse",post),("Independent pickup",earlier.mean(axis=0))):
            errors=[1000*(np.linalg.norm(m/np.array([x,*scale[1:]]))-1) for x in (scale[0],x_gain)]
            out.writerow([name,x_gain,*errors])
            print(name,x_gain,m,errors)
    fig,ax=plt.subplots(figsize=(8,4.6),layout="constrained")
    groups=defaultdict(list)
    for r in rows:
        groups[(r[0],r[1],r[3])].append(r)
    faces=sorted({r[3] for r in rows})
    for j,(idx,label) in enumerate(((8,"Nominal"),(9,"Shelf cube"),(10,"All axes scaled"))):
        for i,face in enumerate(faces):
            values=[np.median([r[idx] for r in group]) for key,group in groups.items() if key[2]==face]
            ax.scatter([i+(j-1)*.16]*len(values),values,label=label if i==0 else None,alpha=.65)
    ax.axhline(0,color=".5",ls=":")
    ax.set(xticks=range(len(faces)),xticklabels=faces,ylabel="Recovered gravity magnitude minus 1 g (mg)",
           xlabel="Dominant gravity axis (within 10°)")
    ax.legend()
    fig.savefig(HERE/"face-coverage.png",dpi=150)
    for face in faces:
        sub=[r for r in rows if r[3]==face]
        print(face,len(sub),sorted(set((r[0],r[1]) for r in sub)),
              np.round(np.median(np.abs(np.array([r[8:] for r in sub])),axis=0),2))


if __name__=="__main__":
    main()
