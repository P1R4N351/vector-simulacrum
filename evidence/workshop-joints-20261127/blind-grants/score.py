"""Score a fixed calm-gated prediction on complete, uncommanded docked grants."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
archive=here.parent.parent/"docked-head-20261126/grants"
grid=np.arange(0,8,.02)


def load(path):
    with path.open() as f:
        rows=list(csv.DictReader(f))
    grant=float(next(r["receipt_s"] for r in rows if r["event"]=="granted"))
    data=np.array([[float(r[k]) for k in ("receipt_s","head_deg","gyro_y","status")]
                   for r in rows if r["event"]=="state"])
    return data[:,0]-grant,data[:,1:3],data[:,3].astype(int)


def learn_template():
    profiles=[]
    for stem in ("pilot","session-1"):
        t,raw,_=load(archive/(stem+".csv"))
        profile=np.column_stack([np.interp(grid,t,column) for column in raw.T])
        profiles.append(profile-np.median(raw[(t>=-1)&(t<0)],axis=0))
    return np.mean(profiles,axis=0)


def main():
    template=learn_template()
    fig,axes=plt.subplots(3,3,figsize=(12,7),sharex=True,sharey="row",layout="constrained")
    summary=[]
    for i,(label,path) in enumerate([("A · archived holdout",archive/"session-2.csv"),
                                     ("B · new grant 1",here/"blind-grant-1.csv"),
                                     ("B · new grant 2",here/"blind-grant-2.csv")]):
        t,raw,flags=load(path)
        before=np.flatnonzero(t<0)[-1]
        calm=bool(flags[before]&1024)
        mask=(t>=0)&(t<8)
        tm=t[mask]
        observed=raw[mask]
        fixed=np.broadcast_to(raw[before],observed.shape)
        wave=fixed+np.column_stack([np.interp(tm,grid,c) for c in template.T])
        gated=wave if calm else fixed
        for k,ax in enumerate(axes[:2,i]):
            ax.plot(tm,observed[:,k],color="black",lw=1,label="Recorded")
            ax.plot(tm,gated[:,k],"--",color="C0",lw=1.4,label="Flag-selected")
            ax.plot(tm,(fixed if calm else wave)[:,k],":",color="C1",label="Other prediction")
        axes[2,i].step(t,(flags&64)!=0,where="post",label="Animating")
        axes[2,i].step(t,(flags&1024)!=0,where="post",label="Calm")
        axes[2,i].set(xlim=(-.5,8),xlabel="Seconds after grant")
        axes[0,i].set_title(label)
        for window in (5.2,8):
            sel=tm<window
            scores=[np.sqrt(np.mean((pred[sel]-observed[sel])**2,axis=0)) for pred in (fixed,wave,gated)]
            summary.append([label,int(calm),window,int(sel.sum()),np.ptp(observed[sel,0]),*np.array(scores).flatten()])
    for ax,label in zip(axes[:,0],("Head angle (°)","Gyro Y (rad/s)","SDK flags")): ax.set_ylabel(label)
    axes[0,0].legend(fontsize=8)
    axes[2,0].legend(fontsize=8)
    fig.savefig(here/"comparison.png",dpi=160)
    with (here/"scores.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("recording pre_calm seconds reports head_range_deg fixed_angle_rms fixed_gyro_rms wave_angle_rms wave_gyro_rms selected_angle_rms selected_gyro_rms".split())
        w.writerows(summary)


if __name__=="__main__":main()
