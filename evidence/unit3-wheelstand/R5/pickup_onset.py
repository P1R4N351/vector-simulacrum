"""Compare two physical pickups on packet receipt clocks with the free-cube replays."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
earlier = here.parents[1] / "unit3-cube-accelerometer"


def read(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf8")


def packet_signal(path, columns, start, scale):
    data = read(path)
    # Preserve the three samples within each packet. Some notifications repeat.
    times = data[columns[0]][::3] * scale - start
    samples = np.stack([data[c] for c in columns[1:]], axis=1).reshape(-1, 3, 3)
    assert np.all(data["slot"].reshape(-1, 3) == [0, 1, 2])
    keep = np.r_[True, np.any(np.diff(samples, axis=0) != 0, axis=(1, 2))]
    times, samples = times[keep], samples[keep] / 8192
    rest = (times >= -1) & (times < 0)
    center = np.median(samples[rest].reshape(-1, 3), axis=0)
    residual = np.linalg.norm(samples - center, axis=2).max(axis=1)
    return times, residual, np.max(residual[rest])


def action_time(path, name, clock):
    with path.open() as f:
        return float(next(r[clock] for r in csv.DictReader(f) if
                          r["action"] == name and r.get("kind", r.get("event")) == "begin"))


def compare():
    first = action_time(earlier/"action-times.csv","pickup","pod_unix_s")
    second = action_time(here/"actions.csv","pickup for inward transport","sec")
    a = packet_signal(earlier/"pickup.csv",["pod_rx_unix_ns","x_counts","y_counts","z_counts"],first,1e-9)
    b = packet_signal(here/"cube.csv.gz",["sec","x","y","z"],second,1)
    prev = read(earlier/"free-pickup.csv")
    prev = prev[prev["initialization"] == "Before command"]
    curr = read(here/"pickup-replay.csv")
    contacts = [prev["time_s"][prev["touches_fork_or_links"] == 1][0],
                curr["sec"][curr["robot_contact"] == 1][0]]
    fig, axes = plt.subplots(2,1,figsize=(9,5.5),sharex=True,layout="constrained")
    with (here/"pickup-onset.csv").open("w") as f:
        out = csv.writer(f,lineterminator="\n")
        out.writerow(["pickup","rest_max_g","gate_g","first_packet_s","confirmation_s","previous_packet_s","sim_contact_s"])
        for label, (t,r,rest), sim, ax in zip(["Earlier pickup","R5"],[a,b],contacts,axes):
            for gate in [.03,.05,.10]:
                # Two successive distinct packets within 0.1 s; avoid bridging long recorder gaps.
                idx = np.flatnonzero((t[:-1]>=0)&(t[:-1]<4)&(r[:-1]>gate)&(r[1:]>gate)&(np.diff(t)<.1))
                i = idx[0]
                out.writerow([label,*[round(float(v),5) for v in (rest,gate,t[i],t[i+1],t[i-1],sim)]])
                if gate == .05:
                    ax.axvline(t[i],color="C1",label="First sustained 0.05 g excursion")
            mask = (t>=-1)&(t<=3.5)
            ax.plot(t[mask],r[mask],".-",lw=.6,ms=3,label="Peak vector change per packet")
            ax.axvline(sim,color="k",ls="--",label="First simulated robot contact")
            ax.axhline(.05,color=".6",ls=":")
            ax.set(title=label,ylabel="Change (g)",yscale="log")
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel("Seconds from pickup command (receiver clock)")
    fig.savefig(here/"pickup-onset.png",dpi=150)


if __name__ == "__main__":
    compare()
