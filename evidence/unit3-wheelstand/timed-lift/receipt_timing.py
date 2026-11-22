"""Bound encoder crossings by the adjacent receipts, and transfer a docked timing offset."""
import csv

import matplotlib.pyplot as plt
import numpy as np

from replay import DURATIONS, HERE, VectorSim, params, replay, rows


def bracket(t, p, level):
    indices = np.flatnonzero((t >= 0) & (p >= level))
    i = indices[0]
    assert i > 0 and p[i-1] < level
    return t[i-1], t[i]


def publication_phases(values, direction, duration):
    """Sample one quantized control-tick replay at every distinct SDK phase."""
    ends = [values["lift_height_min_mm"], values["lift_height_max_mm"]]
    if direction == "lower":
        ends.reverse()
    sim = VectorSim(values, lift_mm=ends[0])
    sim.advance(.5)
    sim.command_lift_height(ends[1], duration_s=duration)
    h = []
    for _ in range(round(2.5/sim.dt)):
        sim.step()
        h.append(sim.reported_now()["lift_mm"])
    progress = (np.array(h)-h[0])/(h[-1]-h[0])
    t = (np.arange(len(h))+1)*sim.dt
    stride = round(values["state_period_ms"]*.001/sim.dt)
    widths = []
    for phase in range(stride):
        q, ts = progress[phase::stride], t[phase::stride]
        widths.append(bracket(ts,q,.9)[1]-bracket(ts,q,.1)[1])
    return min(widths), max(widths)


def measure():
    values = params.values(params.load(model="vector_2"))
    samples = []
    phases = {}
    for label in ("D1", "D2"):
        folder = HERE.parent / label
        state = np.genfromtxt(folder / "state.csv.gz", delimiter=",", names=True)
        events = {(e["action"], e["kind"]): float(e["sec"]) for e in rows(folder / "actions.csv")}
        for move in rows(folder / "moves.csv"):
            if move["timing_valid"] != "1":
                continue
            condition, direction = move["condition"], move["direction"]
            name = " ".join([condition, direction, move["repeat"]])
            start, end = events[name, "begin"], events[name, "end"]
            sel = (state["sec"] >= start-.5) & (state["sec"] <= end+.7)
            t, h = state["sec"][sel]-start, state["lift"][sel]
            progress = (h-np.median(h[t < 0]))/float(move["travel_mm"])
            sim_t, sim_p = replay(values, direction, DURATIONS[condition])
            # Use actual simulated state transitions. Repeated reads of a held state
            # aren't independent sensor samples.
            sim_on, sim_lo, sim_hi = [bracket(sim_t, sim_p, level)[1] for level in (.03,.1,.9)]
            onset, low, high = [bracket(t, progress, level) for level in (.03,.1,.9)]
            key = condition, direction
            if key not in phases:
                phases[key] = publication_phases(values, direction, DURATIONS[condition])
            samples.append(dict(phase_min=phases[key][0], phase_max=phases[key][1], recording=label, stroke=name, onset_lo=onset[0], onset_hi=onset[1],
                                sim_on=sim_on, travel_lo=high[0]-low[1], travel_hi=high[1]-low[0],
                                sim_travel=sim_hi-sim_lo))
    return samples


def main():
    data = measure()
    offset = np.median([(r["onset_lo"]+r["onset_hi"])/2-r["sim_on"] for r in data if r["recording"]=="D1"])
    fig, axes = plt.subplots(1, 2, figsize=(11,7), sharey=True, layout="constrained")
    for i, r in enumerate(data):
        r["shifted_on"] = r["sim_on"]+offset
        for ax, lo, hi, sim in [(axes[0],"onset_lo","onset_hi","sim_on"),
                                (axes[1],"travel_lo","travel_hi","sim_travel")]:
            ax.plot([r[lo],r[hi]], [i,i], linewidth=4, color="#577990", label="Physical receipt bracket" if i==0 else None)
            ax.scatter(r[sim],i, color="#ce5229", s=20, zorder=4, label="Timed sim" if i==0 else None)
        axes[1].plot([r["phase_min"],r["phase_max"]],[i+.17,i+.17],color="#ce5229",
                     label="All sim publication phases" if i==0 else None)
        axes[0].scatter(r["shifted_on"], i, color="#16765a", marker="x", label="Docked offset" if i==0 else None)
    for ax,title in zip(axes, ["Command to 3% travel (s)", "10–90% travel duration (s)"]):
        ax.set_xlabel(title)
        ax.grid(axis="x",alpha=.2)
        ax.legend(fontsize=8)
    axes[0].set_yticks(range(len(data)),[r["recording"]+" "+r["stroke"] for r in data])
    fig.savefig(HERE/"receipt-timing.png",dpi=150)
    with (HERE/"receipt-timing.csv").open("w") as f:
        w=csv.DictWriter(f,fieldnames=data[0],lineterminator="\n")
        w.writeheader()
        w.writerows(data)
    print("Offset s",offset)
    for label in ("D1","D2"):
        sub=[r for r in data if r["recording"]==label]
        for k,lo,hi in [("sim_on","onset_lo","onset_hi"),("shifted_on","onset_lo","onset_hi"),("sim_travel","travel_lo","travel_hi")]:
            print(label,k,sum(r[lo]<=r[k]<=r[hi] for r in sub),len(sub),
                  "median signed midpoint residual s",np.median([(r[lo]+r[hi])/2-r[k] for r in sub]))


if __name__=="__main__":
    main()
