"""Compare docked command replays at the physical SDK receipt times."""
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).parent


def bracket(t, x, threshold):
    indices = np.flatnonzero((x[:-1] < threshold) & (x[1:] >= threshold))
    if not len(indices):
        return None
    i = indices[0]
    estimate = t[i] + (threshold-x[i])*(t[i+1]-t[i])/(x[i+1]-x[i])
    return t[i], estimate, t[i+1]


def main(include_third=False):
    prefixes = ("", "repeat-", "third-") if include_third else ("", "repeat-")
    fig, axes = plt.subplots(2, 2, figsize=(10, 6), layout="constrained", sharex=True)
    hold_fig, hold_axes = plt.subplots(2, len(prefixes), figsize=(4.5*len(prefixes), 6), layout="constrained", sharex=True, sharey="row")
    results = []
    for col, prefix in enumerate(prefixes):
        with (here/(prefix+"joints.csv")).open() as file:
            rows = list(csv.DictReader(file))
        sim = np.genfromtxt(here/(prefix+"canonical-joints.csv"), names=True, delimiter=",")
        assert np.all(sim["on_dock"] == 1)
        commands = [r for r in rows if r["event"] != "state"]
        states = [r for r in rows if r["event"] == "state"]
        times = np.array([float(r["s"]) for r in states])
        for j, cmd in enumerate(commands):
            axis = cmd["event"]
            target = float(cmd["target"])
            if axis == "head" and (not j or commands[j-1]["event"] != "head" or target not in (22,-8)):
                continue
            up = target > 0
            sign = 1 if up else -1
            key = "head_deg" if axis == "head" else "lift_mm"
            cue = float(cmd["s"])
            before = np.flatnonzero(times < cue)[-1]
            end = float(commands[j+1]["s"]) if j+1 < len(commands) else times[-1]
            select = np.arange(before,np.searchsorted(times,end))
            t = times[select]
            x = np.array([float(states[i][key]) for i in select])
            model = sim[key][np.searchsorted(sim["s"],t,side="right")-1]
            lo, hi = (-5,18) if axis == "head" else (36,60)
            thresholds = (lo,hi) if up else (-hi,-lo)
            def transit(trace):
                a,b = [bracket(t,sign*trace,q) for q in thresholds]
                if a is None or b is None:
                    return [np.nan]*3
                return 1000*np.array([max(0,b[0]-a[2]),b[1]-a[1],b[2]-a[0]])
            physical, simulated = transit(x), transit(model)
            results.append([prefix or "first",j,axis,"up" if up else "down",*physical,*simulated])
            if axis == "head" and up:
                for panel, field, ylabel in zip(hold_axes[:,col],("head_deg","gyro_y"),("Head angle (°)","Gyro Y (rad/s)")):
                    physical_trace = [float(states[i][field]) for i in select]
                    simulated_trace = sim[field][np.searchsorted(sim["s"],t,side="right")-1]
                    color = "C0" if j == 1 else "C3"
                    panel.plot(t-cue,physical_trace,".-",color=color,label=f"Physical raise {1 if j==1 else 2}")
                    panel.plot(t-cue,simulated_trace,"--",color=color,label="Simulated",lw=1)
                    panel.set_ylabel(ylabel)
                hold_axes[0,col].set_title(["First recording", "Second recording", "Third recording"][col])
            if prefix == prefixes[-1]:
                ax = axes[int(axis=="lift"),int(not up)]
                ax.plot(t-cue,x,".-",color="C0",lw=1,label="Physical")
                ax.plot(t-cue,model,".--",color="C1",lw=1,label="Canonical")
                ax.set(title=f"{axis.capitalize()} {'raise' if up else 'lower'}",xlim=(-.08,.65),
                       ylabel="Degrees" if axis=="head" else "Millimeters")
    handles, labels = axes[0,0].get_legend_handles_labels()
    axes[0,0].legend(handles[:2], labels[:2])
    for ax in axes[-1]: ax.set_xlabel("Seconds from command")
    hold_axes[0,0].legend(fontsize=7)
    for panel in hold_axes[-1]: panel.set_xlabel("Seconds from head raise command")
    hold_fig.savefig(here/("three-head-holds.png" if include_third else "head-hold.png"),dpi=160)
    fig.savefig(here/("third-canonical.png" if include_third else "repeat-canonical.png"),dpi=160)
    with (here/("three-canonical-transits.csv" if include_third else "canonical-transits.csv")).open("w") as file:
        w = csv.writer(file,lineterminator="\n")
        w.writerow("recording cue axis direction real_low_ms real_ms real_high_ms sim_low_ms sim_ms sim_high_ms".split())
        w.writerows(results)


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--include-third", action="store_true")
    main(parser.parse_args().include_third)
