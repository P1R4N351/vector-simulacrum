"""Score captures against the workshop A grant waveform on the uncommanded interval."""
import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

root=Path(__file__).parent


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def event(rows,name):
    return next(float(r.get("s") or r["receipt_s"]) for r in rows if r["event"]==name)


def main():
    grid=np.arange(0,5.2,.02)
    profiles=[]
    for name in ("pilot","session-1"):
        rows=read(root.parent.parent/"docked-head-20261126/grants"/(name+".csv"))
        a=np.array([[float(r[k]) for k in ("receipt_s","head_deg","gyro_y")] for r in rows if r["event"]=="state"])
        a[:,0]-=event(rows,"granted")
        baseline=np.median(a[(a[:,0]>=-1)&(a[:,0]<0),1:],axis=0)
        profiles.append(np.column_stack([np.interp(grid,a[:,0],a[:,k+1]-baseline[k]) for k in range(2)]))
    template=np.mean(profiles,axis=0)
    fig,axes=plt.subplots(3,4,figsize=(13,7),layout="constrained",sharex="col",sharey="row")
    result=[]
    for i,delay in enumerate((1,7,7,1)):
        rows=read(root/f"grant-delay-{i+1}.csv")
        grant=event(rows,"grant")
        states=[r for r in rows if r["event"]=="state"]
        t,head,gyro,flags,dock,wl,wr=np.array([[float(r[k]) for k in ("s","head_deg","gyro_y","status","on_dock","wl","wr")] for r in states]).T
        t-=grant
        commands=[float(r["s"])-grant for r in rows if r["event"]=="command"]
        returns=[float(r["s"])-grant for r in rows if r["event"]=="return"]
        assert len(commands)==3 and min(dock)==1
        hold=(t>=returns[1])&(t<commands[2])
        anim=(flags.astype(int)&64)!=0
        calm=(flags.astype(int)&1024)!=0
        cols=[]
        for ax,y in zip(axes[:2,i],(head,gyro)):
            ax.plot(t,y,".-",ms=2)
        if delay==7:
            baseline=np.median(np.stack([head,gyro],axis=1)[(t>=-1)&(t<0)],axis=0)
            sel=(t>=0)&(t<5.2)
            for k,(ax,y) in enumerate(zip(axes[:2,i],(head,gyro))):
                prediction=np.interp(t[sel],grid,template[:,k])+baseline[k]
                ax.plot(t[sel],prediction,"--",color="C1",label="Other v1 prediction")
                latest = y[t<0][-1]
                cols.extend([np.sqrt(np.mean((y[sel]-baseline[k])**2)),np.sqrt(np.mean((y[sel]-prediction)**2)),
                             np.sqrt(np.mean((y[sel]-latest)**2)),np.sqrt(np.mean((y[sel]-prediction+baseline[k]-latest)**2))])
        for ax in axes[:,i]:
            for cue in commands: ax.axvline(cue,lw=.6,color=".55")
            ax.axvspan(returns[1],commands[2],color="C2",alpha=.10)
        axes[2,i].step(t,anim,where="post",label="Animating")
        axes[2,i].step(t,calm,where="post",label="Calm")
        axes[0,i].set_title(f"Session {i+1} · {delay} s wait")
        axes[2,i].set_xlabel("Seconds after grant")
        result.append([i+1,delay,len(t),calm[t<0].sum(),sum(t<0),int(sum(anim[hold])),int(sum(hold)),max(head[hold]),min(head[hold]),np.ptp(head[hold]),max(abs(gyro[hold])),max(np.max(abs(wl)),np.max(abs(wr))),*cols])
    axes[2,0].legend(fontsize=8)
    axes[0,1].legend(fontsize=8)
    for ax,unit in zip(axes[:,0],("Head angle (°)","Gyro Y (rad/s)","SDK flags")): ax.set_ylabel(unit)
    fig.savefig(root/"grant-delay.png",dpi=150)
    with (root/"scores.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("session delay_s packets passive_calm passive_packets hold_anim hold_packets hold_high_deg hold_low_deg hold_range_deg peak_gyro max_wheel_mm_s stationary_angle_rmse template_angle_rmse last_angle_rmse last_plus_template_angle_rmse stationary_gyro_rmse template_gyro_rmse last_gyro_rmse last_plus_template_gyro_rmse".split())
        for row in result:w.writerow(row+[""]*(20-len(row)))


if __name__=="__main__":main()
