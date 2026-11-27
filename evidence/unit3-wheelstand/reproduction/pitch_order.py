"""Compare first simulated cube contact to errors at actual SDK pitch sample times."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

import replay
from wheelstand_load import FirstLoads

HERE = Path(__file__).resolve().parent


def main(fixture):
    v = replay.params.values(replay.params.load(model="vector_2"), {"tip_supports_driven":1})
    m = replay.build(v,fixture)
    m.opt.timestep = v["sim_timestep_ms"]*.001/4
    fig, axes = plt.subplots(2,2,figsize=(10,6),layout="constrained",sharex="col")
    with (HERE/"pitch-order.csv").open("w",newline="") as f:
        out = csv.writer(f,lineterminator="\n")
        out.writerow(["run","case","release_s","onset_s","first_fork_load_s","first_box_load_s","sdk_samples_before_box","last_pre_box_sample_s","last_pre_box_residual_deg","max_abs_pre_box_deg","first_pair_over_5deg_s","first_pair_over_10deg_s"])
        for col,name in enumerate(("W4","W8")):
            tr = replay.Trial(name)
            s = tr.state
            ax,err = axes[:,col]
            for case,(r,o) in enumerate(((r,o) for r in (3.5,3.8) for o in (tr.early,tr.late)),1):
                start = r+tr.shift
                loads = FirstLoads(m)
                trace,*_ = replay.run(m,v,tr,start,o,"servo:60",17,on_step=loads.step)
                ts = s["sec"]-tr.zero
                mask = (ts>=trace[0,0]+m.opt.timestep)&(ts<tr.late+.8)
                ts = ts[mask]
                actual = np.rad2deg(s["pitch"][mask])
                delta = np.interp(ts,trace[:,0]+m.opt.timestep,trace[:,1])-actual
                box_t = loads.times.get("box",np.nan)
                pre = ts<box_t
                def crossing(thresh):
                    bad = abs(delta)>thresh
                    ids = np.flatnonzero(bad[:-1]&bad[1:])
                    return ts[ids[0]] if len(ids) else np.nan
                out.writerow([name,case,start,o,loads.times.get("fork"),box_t,sum(pre),ts[pre][-1] if any(pre) else None,delta[pre][-1] if any(pre) else None,max(abs(delta[pre]),default=np.nan),crossing(5),crossing(10)])
                color = f"C{case-1}"
                ax.plot(trace[:,0]-tr.late,trace[:,1],color=color,label=f"Timing {case}")
                err.plot(ts-tr.late,delta,".-",color=color,ms=4)
                for panel in (ax,err):
                    panel.axvline(box_t-tr.late,color=color,lw=.8,ls=":")
            ax.plot(s["sec"]-tr.zero-tr.late,np.rad2deg(s["pitch"]),"k.-",ms=4,label="SDK")
            ax.set_title(name)
            err.axhline(0,color="black",lw=.6)
            err.set_xlabel("Seconds from last raised-lift sample")
            err.set_xlim(-.52,.65)
    axes[0,0].set_ylabel("Pitch (°)")
    axes[1,0].set_ylabel("Sim minus SDK (°)")
    axes[0,0].legend(fontsize=7)
    fig.suptitle("Dotted lines mark first loaded chassis–cube contact in each replay")
    fig.savefig(HERE/"pitch-order.png",dpi=160)


if __name__=="__main__":
    main(sys.argv[1])
