"""Compare native wheel-driven Vector 2.0 replays; calibrate only on the first pair."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from replay_measured_turn import run

ROOT = Path(__file__).resolve().parent / "evidence"
DATA = ROOT / "hw-rec-20261119"
COUNT_MM = .96 * 29 * .25 * np.pi / 172.3
LIFT_MM = 30.64501190185547
TRAIN = ("ccw_1", "cw_1")
TEST = ("ccw_2", "cw_2")
WINDOWS = {"steady": (-.85,-.25), "brake": (-.10,.10), "tail": (0,.10)}


def delayed(clock, signal, times, delay_ms):
    return np.interp(times-delay_ms/1000, clock, signal)


def rms(cases, key, window):
    low, high = WINDOWS[window]
    errors = []
    for case in cases:
        times = case["t"]
        mask = (times >= low) & (times <= high)
        errors.extend((case[key][mask] - case["measured"][mask])**2)
    return np.sqrt(np.mean(errors))


def simulate(rows):
    def col(name):
        return np.array([float(row[name]) for row in rows])
    t = col("ms_from_last_edge") / 1000
    w = np.column_stack([col("left_mmps"), col("right_mmps")])
    if np.any(w[0]) or np.any(np.diff(t)<=0):
        raise ValueError("Replay needs a stationary start and ordered native timestamps")
    pose = [{"head":col("head_rad")[0], "lift":LIFT_MM}]
    result = {"t":t, "measured":np.degrees(col("head_gz")), "w":w}
    for span in (42,48):
        clock, output = run(pose,t,w,span,dense=True)
        result[str(span)] = delayed(clock,output[:,0],t,17)
        result[f"tracking{span}"] = np.max(abs(output[:,1:] -
            np.column_stack([np.interp(clock,t,wheel) for wheel in w.T])))
        if span == 42:
            result.update(clock=clock, signal=output[:,0], raw=delayed(clock,output[:,0],t,0))
    # Independent input check. Differences are interval averages with midpoint
    # timestamps. Convert shaft counts to tread mm and normalize right polarity.
    counts = np.column_stack([col("left_count"), -col("right_count")])
    rates = COUNT_MM*np.diff(counts,axis=0)/np.diff(t)[:,None]
    mid = (t[1:]+t[:-1])/2
    c, output = run(pose,mid,rates,42,dense=True)
    result["count42"] = delayed(c,output[:,0],t,17)
    return result


def main():
    with (DATA/"unit2-native-replay-input.csv").open() as f:
        rows = list(csv.DictReader(f))
    cases = {label:simulate([r for r in rows if r["turn"]==label]) for label in TRAIN+TEST}
    training = [cases[label] for label in TRAIN]
    # One global time shift. Fit the brake window, then apply through the entire turn.
    def loss(ms):
        for case in training:
            case["trial"] = delayed(case["clock"],case["signal"],case["t"],ms)
        return rms(training,"trial","brake")
    lag = min(range(41),key=loss)
    print("Training lag (ms)",lag)
    for label,case in cases.items():
        case["fit"] = delayed(case["clock"],case["signal"],case["t"],lag)
        print(label,"tracking (mm/s)",case["tracking42"],case["tracking48"])
    keys = ("48","42","raw","count42","fit")
    for label,values in [(k,[v]) for k,v in cases.items()] + [
            ("training",training),("held-out",[cases[k] for k in TEST])]:
        for window in WINDOWS:
            print(label,window,*(round(rms(values,k,window),2) for k in keys))

    with (DATA/"native-replay-predictions.csv").open("w") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow(("turn","ms_from_last_edge","head_gyro_dps","span48_17ms","span42_17ms",
                         "span42_unshifted","span42_count_input_17ms","span42_training_lag"))
        for label,case in cases.items():
            for row in zip(np.rint(case["t"]*1000).astype(int),case["measured"],*(case[k] for k in keys)):
                writer.writerow((label,*row))
    fig,axes=plt.subplots(3,4,figsize=(14,7.8),sharex=True,sharey="row")
    for i,(label,case) in enumerate(cases.items()):
        t=case["t"]*1000
        view=(t>=-220)&(t<=120)
        sign=1 if label.startswith("ccw") else -1
        ax,err,enc=axes[:,i]
        ax.set_title(label.upper().replace("_"," ")+(" · fit" if label in TRAIN else " · held out"))
        for key,color,style,name in [
            ("measured","#20252e","-","Native head gyro"),("48","#ad744c","--","48 mm · 17 ms"),
            ("42","#176397","-","42 mm · 17 ms"),("fit","#19846e",":",f"42 mm · {lag} ms fit")]:
            ax.plot(t[view],sign*case[key][view],style,color=color,label=name,lw=1.8)
            if key!="measured":
                err.plot(t[view],sign*(case[key]-case["measured"])[view],style,color=color,lw=1.6)
        for j,name,color in [(0,"Left","#875998"),(1,"Right","#5d803e")]:
            enc.plot(t[view],abs(case["w"][view,j]),color=color,label=name,lw=1.2)
        for pane in axes[:,i]:
            pane.axvline(0,color=".4",lw=.9)
            pane.grid(alpha=.17)
            pane.spines[["top","right"]].set_visible(False)
    axes[0,0].set_ylabel("Turn-aligned head gyro (°/s)")
    axes[1,0].set_ylabel("Prediction − gyro (°/s)")
    axes[2,0].set_ylabel("|Wheel speed| (mm/s)")
    axes[2,0].legend(fontsize=8)
    fig.legend(*axes[0,0].get_legend_handles_labels(),loc="upper center",ncol=4,bbox_to_anchor=(.5,.946))
    fig.suptitle("Vector 2.0  ·  replay of four native 100 mm/s stops",weight="bold",fontsize=16)
    fig.supxlabel("Milliseconds from last detected wheel edge")
    fig.subplots_adjust(top=.842,bottom=.10,left=.075,right=.985,wspace=.13,hspace=.2)
    fig.savefig(ROOT/"physics-plots"/"native-span-replay.png",dpi=160)

    fig, axes = plt.subplots(1,4,figsize=(13,3.8),sharex=True,sharey=True)
    for ax,(label,case) in zip(axes,cases.items()):
        t = case["t"]*1000
        view = (t>=-80)&(t<=100)
        sign = 1 if label.startswith("ccw") else -1
        for key,color,name in [("measured","#20252e","Native head gyro"),
                               ("42","#176397","Reported speed input"),
                               ("count42","#b34f36","Count-derived input")]:
            ax.plot(t[view],sign*case[key][view],color=color,label=name,lw=1.6)
        zero = np.flatnonzero(np.any(case["w"],axis=1))[-1]+1
        ax.axvline(0,color=".35",lw=1)
        ax.axvline(t[zero],color=".4",ls="--",lw=1)
        ax.set_title(label.upper().replace("_"," ")+(" · fit" if label in TRAIN else " · held out"))
        ax.grid(alpha=.17)
    axes[0].set_ylabel("Turn-aligned head gyro (°/s)")
    fig.suptitle("Encoder input comparison  ·  42 mm and 17 ms throughout",weight="bold",fontsize=15)
    fig.legend(*axes[0].get_legend_handles_labels(),loc="upper center",ncol=3,bbox_to_anchor=(.5,.92))
    fig.supxlabel("Milliseconds from last encoder edge (solid)  ·  both reported speeds zero (dashed)")
    fig.subplots_adjust(top=.73,bottom=.2,left=.07,right=.98,wspace=.15)
    fig.savefig(ROOT/"physics-plots"/"native-count-replay.png",dpi=160)


if __name__=="__main__":
    main()
