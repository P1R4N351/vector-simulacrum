"""Evaluate the already-frozen grant predictor on sleep-conditioned captures."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from score import learn_template, grid, load

here=Path(__file__).resolve().parent


def main():
    runs=[here/f"{prefix}-{i}.csv" for prefix in ("blind-grant","calm-cross") for i in (1,2)]
    template=learn_template()
    fig,axes=plt.subplots(3,len(runs),squeeze=False,figsize=(14,6),layout="constrained")
    rows=[]
    for col,path in enumerate(runs):
        t,y,flags=load(path)
        prior=np.flatnonzero(t<0)[-1]
        sel=(t>=0)&(t<8)
        assert sel.sum()>=120
        base=y[prior]
        a=(flags&64)!=0
        calm=(flags&1024)!=0
        estimates=base+np.column_stack([np.interp(t[sel],grid,c) for c in template.T])
        scores=[]
        for k,label in enumerate(("Head angle (°)","Gyro Y (rad/s)")):
            ax=axes[k,col]
            ax.plot(t,y[:,k],color="black",label="Measured")
            ax.plot(t[sel],estimates[:,k],"--",color="C1",label="Frozen waveform")
            ax.axhline(base[k],color="C0",ls=":",label="Stationary")
            ax.set_ylabel(label)
            scores.extend([np.sqrt(np.mean((y[sel,k]-base[k])**2)),np.sqrt(np.mean((y[sel,k]-estimates[:,k])**2))])
        axes[2,col].step(t,a,where="post",label="Animating")
        axes[2,col].step(t,calm,where="post",label="Calm")
        for ax in axes[:,col]:
            ax.set_xlim(-2,8)
            ax.axvline(0,color=".6",lw=.7)
        axes[2,col].set_xlabel("Seconds after grant")
        axes[0,col].set_title(f"{'Calm' if calm[prior] else 'Awake'} · grant {col%2+1}")
        rows.append([path.stem,int(calm[prior]),int(sum(sel)),int(sum(a&sel)),base[0],np.ptp(y[sel,0]),*scores])
    axes[0,0].legend(fontsize=8)
    axes[2,0].legend(fontsize=8)
    fig.savefig(here/"crossover.png",dpi=160)
    with (here/"crossover-scores.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("capture pre_calm packets animating pre_deg head_range_deg fixed_angle_rmse wave_angle_rmse fixed_gyro_rmse wave_gyro_rmse".split())
        w.writerows(rows)


if __name__=="__main__":main()
