"""Compare simulated contact through cube firmware and engine grouping to physical SDK callbacks."""
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import replay
from simulacrum.cube import TapBroadcasts

here = Path(__file__).resolve().parent


def clock(candidates, phase):
    listener = TapBroadcasts()
    for index, t in enumerate(candidates):
        listener.receive(t + phase, index)
    if candidates:
        listener.advance(candidates[-1] + phase + 1)
    return np.array([t - phase for t, _ in listener.events])


def observed(trial):
    packets = trial.cube[::3]
    times = packets["sec"] - trial.zero
    lo, hi = trial.late - .3, min(trial.late + 1.2, times[-1])
    packet_times = times[1:][np.diff(packets["tap"]) != 0]
    with (trial.dir / "events.csv").open() as f:
        callbacks = [float(r["sec"])-trial.zero for r in csv.DictReader(f) if r["event"] == "object_tapped"]
    return lo, hi, [t for t in packet_times if lo <= t <= hi], [t for t in callbacks if lo <= t <= hi]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("fixture")
    args = parser.parse_args()
    fig, axes = plt.subplots(4, 2, figsize=(11,12), layout="constrained")
    with (here/"contact-events.csv").open("w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["run","model","case","release_s","onset_s","source","candidates_s","sdk_s","phase_ms","broadcast_s"])
        for col, run in enumerate(("W4","W8")):
            trial = replay.Trial(run)
            lo, hi, recorded, callbacks = observed(trial)
            for row, (variant, driven, torque) in enumerate([("Fixed / 30",0,30),("Driven / 30",1,30),("Fixed / 60",0,60),("Driven / 60",1,60)]):
                ax = axes[row,col]
                v = replay.params.values(replay.params.load(model="vector_2"),{"tip_supports_driven":driven})
                model = replay.build(v,args.fixture)
                model.opt.timestep = v["sim_timestep_ms"] * .001 / 4
                cases = [(s+trial.shift,o) for s in (3.5,3.8) for o in (trial.early,trial.late)]
                series = [(0,None,None,"Physical packets",recorded)]
                for n,(start,onset) in enumerate(cases,1):
                    candidates = []
                    replay.run(model,v,trial,start,onset,f"servo:{torque}",17,tap_times=candidates)
                    series.append((n,start,onset,"MuJoCo contact",[t for t in candidates if lo<=t<=hi]))
                for n,start,onset,source,candidates in series:
                    ax.scatter(callbacks,[n]*len(callbacks),color="black",marker="|",s=200,
                               label="Physical SDK" if n==0 else None)
                    for ms in range(60):
                        out = clock(candidates,ms/1000)
                        out = out[out <= hi]  # common capture ends before some late publications
                        to_text = lambda times: " ".join(f"{x:.6f}" for x in times)
                        w.writerow([run,variant,n,start,onset,source,to_text(candidates),to_text(callbacks),ms,to_text(out)])
                        ax.scatter(out,np.full(len(out),n+.2),s=4,alpha=.2,color="#247799",
                                   label="Predicted broadcast" if n==0 and ms==0 else None)
                ax.set(title=f"{run} · {variant.lower()} mN m",
                       yticks=range(5),yticklabels=["Physical packets","Early / early","Early / late","Late / early","Late / late"],
                       xlabel="Seconds after wheelstand command")
                ax.invert_yaxis()
    axes[0,1].legend(fontsize=8)
    fig.savefig(here/"contact-events.png",dpi=165)

if __name__ == "__main__":
    main()
