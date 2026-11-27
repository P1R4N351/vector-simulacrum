"""Register wheelstand impacts against the chassis rising through a pitch landmark."""
import csv
from itertools import product
import sys

import matplotlib.pyplot as plt
import numpy as np

from impact_slots import here, replay, signed


def summarize(rows, ax, run, source, origin, t, pitch, left, message_time, signal):
    active = (message_time >= origin - .14) & (message_time <= origin + .8)
    targets = np.flatnonzero(active)
    peak = targets[np.argmax(abs(signal[active]))]
    impact = message_time[peak]
    span = (t >= origin - .3) & (t <= origin + .8)
    st, sp, sw = t[span], pitch[span], left[span]
    color = "black" if source == "physical" else "tab:blue"
    alpha = 1 if source == "physical" else .45
    for panel, y in zip(ax[:2], (sp, sw)):
        panel.plot(st-origin, y, color=color, alpha=alpha)
    values = signal.reshape(-1, 3)
    idx = abs(values).argmax(axis=1)
    ax[2].plot(message_time[::3]-origin, values[np.arange(len(values)),idx], ".-", color=color, alpha=alpha)
    ax[2].scatter(impact-origin, signal[peak], color=color, marker="x", s=60)
    posture = [np.interp(impact, st, sp), np.interp(impact, st, sw), np.max(sp)]
    for threshold in (10, 20, 30):
        crossings = np.flatnonzero((sp[:-1] < threshold) & (sp[1:] >= threshold))
        if not len(crossings):
            rows.append([run, source, threshold, impact-origin, signal[peak], *[""]*4, *posture])
            continue
        i = crossings[0]
        before, after = st[i:i+2]
        cross = np.interp(threshold, sp[i:i+2], st[i:i+2])
        rows.append([run, source, threshold, impact-origin, signal[peak], before-origin, after-origin,
                     1000*(impact-cross), 1000*(after-before), *posture])


def main(fixture):
    rows=[]
    fig, axes = plt.subplots(3, 2, figsize=(10,7), sharex=True, sharey="row", layout="constrained")
    v = replay.params.values(replay.params.load(model="vector_2"), {"tip_supports_driven":0})
    m = replay.build(v,fixture)
    m.opt.timestep = v["sim_timestep_ms"]*.001/4
    baseline = np.array([0,0,v["cube_accel_scale"][2]]) + np.array(v["cube_accel_bias_mg"])*.001
    for col, name in enumerate(("W4","W8")):
        tr = replay.Trial(name)
        s, c = tr.state, tr.cube
        times = c["sec"] - tr.zero
        raw = np.stack([c[k] for k in "xyz"],axis=1)/8192
        rest = np.median(raw[(times >= -.8)&(times < 0)],axis=0)
        summarize(rows,axes[:,col],name,"physical",tr.late,s["sec"]-tr.zero,np.degrees(s["pitch"]),s["left"],
                  times,signed(raw,rest))
        for case,(start,onset) in enumerate(product((3.5,3.8),(tr.early,tr.late)),1):
            chips=[]
            def keep(r,t):
                if not chips: chips.append(r.sensor)
            start += tr.shift
            trace,*_ = replay.run(m,v,tr,start,onset,"servo:30",17,on_step=keep)
            msgs = chips[0].messages
            st = np.repeat([start+t for t,_ in msgs],3)
            words = np.concatenate([np.frombuffer(p,dtype="<i2",offset=2).reshape(3,3) for _,p in msgs])/8192
            summarize(rows,axes[:,col],name,f"sim {case}",tr.late,trace[:,0],trace[:,1],trace[:,5],st,signed(words,baseline))
        axes[0,col].set_title(name+" · physical (black), four replays (blue)")
        for a in axes[:,col]:
            a.axvline(0,color=".6",lw=.8)
            a.set_xlim(-.26,.43)
        axes[2,col].set_xlabel("Seconds from last raised-lift sample")
    for ax,label in zip(axes[:,0],("Chassis pitch (°)","Left wheel (mm/s)","Cube packet peak (signed g)")):
        ax.set_ylabel(label)
    fig.savefig(here/"impact-posture.png",dpi=150)
    with (here/"impact-posture.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow("run source threshold_deg impact_s signed_g crossing_low_s crossing_high_s impact_minus_cross_ms bracket_ms pitch_at_impact_deg left_at_impact_mmps max_pitch_deg".split())
        writer.writerows(rows)


if __name__=="__main__": main(sys.argv[1])
