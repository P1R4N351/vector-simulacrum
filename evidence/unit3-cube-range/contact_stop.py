"""Score the local charger stop over the first-contact SDK sampling interval."""
import csv
import io
import subprocess

import matplotlib.pyplot as plt
import numpy as np

from return_braking import ROOT, states, table, wheel_travel


def replay(mode, speeds):
    output = subprocess.check_output(["node", str(ROOT / "return_replay.mjs"), mode,
                                      *map(str, sorted(set(speeds)))], text=True)
    return np.genfromtxt(io.StringIO(output), names=True, delimiter=",", dtype=None, encoding="utf8")


def main():
    trials = table("return-braking.csv")
    contact = table("return-scores.csv")
    runs = [replay(option, trials["approach_mmps"]) for option in ("--wheels", "--contact-stop")]
    fig, axes = plt.subplots(1, len(trials), figsize=(14, 3.5), layout="constrained", sharey=True)
    scores = []
    for ax, trial in zip(axes, trials):
        n = trial["session"]
        timing = contact[contact["session"] == n][0]
        window = -timing["last_off_rel_s"]
        rows, _ = states(n)
        times = np.array([float(r["receipt_s"]) for r in rows]) - timing["first_on_s"]
        speeds = np.array([(float(r["left_mmps"])+float(r["right_mmps"]))/2 for r in rows])
        observed = (times >= -.2) & (times < .25)
        ax.scatter(times[observed], -speeds[observed], c="black", s=13, label="Recorded wheels", zorder=3)
        for sim, label, color in zip(runs, ["Queued", "Local"], ["#a4501f", "#147487"]):
            cycle = sim[(sim["speed"] == trial["approach_mmps"]) & np.isin(sim["phase"], ["return", "seat"])]
            t = cycle["t_s"] - cycle["t_s"][np.flatnonzero(cycle["on"])[0]]
            w = (cycle["left_mmps"] + cycle["right_mmps"]) / 2
            x = cycle["x_mm"]
            tread = (cycle["tread_left_mm"] + cycle["tread_right_mm"])/2
            frames = np.array([trial["before_s"], trial["after_s"]])
            phases = np.linspace(0, window, 21)
            curves = []
            travel = []
            for phase in phases:
                shifted = frames + phase
                xyz = np.interp(shifted, t, x)
                travel.append(xyz[0]-xyz[1])
                curves.append(-np.interp(times[observed]+phase,t,w))
            curves = np.array(curves)
            ax.plot(times[observed],curves[10],color=color,label=label)
            ax.fill_between(times[observed],curves.min(axis=0),curves.max(axis=0),color=color,alpha=.15)
            raw = np.interp(frames,t,tread)
            zero = np.flatnonzero((t>=0)&(cycle["left_mmps"]==0)&(cycle["right_mmps"]==0))[0]
            scores.append([n,label,window,trial["camera_mm"],travel[0],travel[10],min(travel),max(travel),
                           raw[0]-raw[1],wheel_travel(t,w,*frames),t[zero]])
        ax.axvline(0,color="grey",lw=.8)
        ax.set(title=n,xlabel="Time from first on-report (s)")
    axes[0].set_ylabel("Mean backing wheel speed (mm/s)")
    axes[-1].legend(fontsize=8)
    fig.savefig(ROOT / "contact-stop.png", dpi=160)
    with (ROOT / "contact-stop.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("session stop last_report_gap_s camera_mm zero_phase_mm midpoint_mm min_mm max_mm tread_zero_phase_mm encoder_zero_phase_mm both_zero_s".split())
        w.writerows(scores)


if __name__ == "__main__":
    main()
