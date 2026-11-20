"""A once-per-stop encoder switch for the recorded 100 mm/s Vector 2.0 turns."""
import csv

import matplotlib.pyplot as plt
import numpy as np

from replay_native_stops import COUNT_MM, DATA, ROOT, LIFT_MM, TRAIN, TEST, WINDOWS, delayed, rms
from replay_measured_turn import run


def switch_index(wheels):
    speed = np.mean(abs(wheels), axis=1)
    armed = False
    high = low = 0
    for i, value in enumerate(speed):
        high = high + 1 if value >= 90 else 0
        if high >= 8:
            armed = True
        low = low + 1 if armed and value < 85 else 0
        if low == 2:
            return i
    raise ValueError("No sustained turn followed by a braking threshold crossing")


def replay(rows, reference):
    def col(key):
        return np.array([float(r[key]) for r in rows])
    t = col("ms_from_last_edge")/1000
    w = np.column_stack([col("left_mmps"),col("right_mmps")])
    counts = np.column_stack([col("left_count"),-col("right_count")])
    end = t[1:]
    rate = COUNT_MM*np.diff(counts,axis=0)/np.diff(t)[:,None]
    idx = switch_index(w)
    sw = t[idx]
    result = {"t":t,"switch":sw,"measured":np.degrees(col("head_gz")),"w":w}
    assert np.array_equal(col("ms_from_last_edge"),[float(r["ms_from_last_edge"]) for r in reference])
    assert np.allclose(result["measured"],[float(r["head_gyro_dps"]) for r in reference])
    for k,ref in [("reported","span42_17ms"),("counts","span42_count_input_17ms")]:
        result[k] = np.array([float(r[ref]) for r in reference])
    pose = [{"head":col("head_rad")[0],"lift":LIFT_MM}]
    c, output = run(pose,end,rate,42,dense=True)
    result["count_end"] = delayed(c,output[:,0],t,17)
    for key,count_times,decision in [("hybrid",(end+t[:-1])/2,idx),("end_time",end,idx),
                                     ("early",end,idx-1),("late",end,idx+1)]:
        # Leave all reported-speed knots through the decision intact. Use only
        # subsequent count intervals; the same interpolation as each baseline
        # joins the last reported-speed knot to the first count-speed knot.
        times = np.r_[t[:decision+1],count_times[decision:]]
        inputs = np.vstack([w[:decision+1],rate[decision:]])
        clock,out = run(pose,times,inputs,42,dense=True)
        result[key] = delayed(clock,out[:,0],t,17)
        target = np.column_stack([np.interp(clock,times,side) for side in inputs.T])
        if np.max(abs(target-out[:,1:])) > .011:
            raise ValueError("Wheel tracking tolerance exceeded")
        if key=="end_time":
            result["no_lag"] = delayed(clock,out[:,0],t,0)
            result["input"] = np.column_stack([np.interp(t,times,side) for side in inputs.T])
    if any(not np.allclose(result[k][t<=sw],result["reported"][t<=sw],atol=1e-7,rtol=0)
           for k in ("hybrid", "end_time")):
        raise ValueError("Hybrid altered the pre-switch gyro trace")
    return result


def read(name):
    with (DATA/name).open() as f:
        return list(csv.DictReader(f))


def main():
    rows = read("unit2-native-replay-input.csv")
    reference = read("native-replay-predictions.csv")
    cases = {k:replay([r for r in rows if r["turn"]==k],[r for r in reference if r["turn"]==k])
             for k in TRAIN+TEST}
    keys=("reported","counts","hybrid","end_time","count_end","early","late","no_lag")
    for label,group in [(k,[v]) for k,v in cases.items()]+[("held-out",[cases[k] for k in TEST])]:
        for phase in WINDOWS:
            print(label,phase,*(round(rms(group,k,phase),2) for k in keys))
    with (DATA/"hybrid-encoder-replay.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow(("turn","ms_from_last_edge","ms_from_switch","left_input_mmps","right_input_mmps",
                         "measured_head_dps","reported","counts","hybrid_midpoint","hybrid_end_time","count_only_end_time","switch_early","switch_late","hybrid_no_lag"))
        for k,c in cases.items():
            print(k,"switch ms",round(c["switch"]*1000))
            for row in zip(np.rint(c["t"]*1000).astype(int),np.rint((c["t"]-c["switch"])*1000).astype(int),
                           *c["input"].T,c["measured"],*(c[key] for key in keys)):
                writer.writerow((k,*row))
    fig,axes=plt.subplots(3,4,figsize=(14,7.8),sharex=True,sharey="row")
    for i,(label,c) in enumerate(cases.items()):
        t=c["t"]*1000
        v=(t>=-160)&(t<=110)
        sign=1 if label.startswith("ccw") else -1
        ax,err,inp=axes[:,i]
        ax.set_title(label.upper().replace("_"," ")+(" · development" if label in TRAIN else " · held out"))
        for key,style,color,name in [("measured","-","#232632","Native gyro"),
                                     ("reported","--","#376a9f","Reported only"),
                                     ("counts",":","#bc6a42","Counts only"),
                                     ("end_time","-","#00806f","Hybrid · interval end")]:
            ax.plot(t[v],sign*c[key][v],style,color=color,lw=1.7,label=name)
            if key!="measured":
                err.plot(t[v],sign*(c[key][v]-c["measured"][v]),style,color=color,lw=1.5)
        for w,name,color in [(c["w"],"Reported","#376a9f"),(c["input"],"Hybrid","#00806f")]:
            inp.plot(t[v],np.mean(abs(w[v]),axis=1),color=color,label=name)
        for pane in axes[:,i]:
            pane.axvline(c["switch"]*1000,color="#985389",ls="--",lw=1)
            pane.axvline(0,color=".35",lw=.9)
            pane.grid(alpha=.16)
            pane.spines[["top","right"]].set_visible(False)
    axes[0,0].set_ylabel("Turn-aligned head gyro (°/s)")
    axes[1,0].set_ylabel("Prediction − gyro (°/s)")
    axes[2,0].set_ylabel("Mean |wheel speed| (mm/s)")
    axes[2,0].legend(fontsize=8)
    fig.suptitle("Vector 2.0  ·  switch from reported speeds to encoder counts during braking",weight="bold")
    fig.legend(*axes[0,0].get_legend_handles_labels(),loc="upper center",ncol=4,bbox_to_anchor=(.5,.946))
    fig.supxlabel("Milliseconds from final wheel edge (solid)  ·  encoder switch (dashed)")
    fig.subplots_adjust(top=.84,bottom=.10,left=.08,right=.985,wspace=.13,hspace=.2)
    fig.savefig(ROOT/"physics-plots"/"hybrid-encoder-replay.png",dpi=160)


if __name__=="__main__":
    main()
