"""Estimate effective encoder-to-gyro delay on unit 1, transfer it to unit 2."""
import csv
import json
from pathlib import Path

import numpy as np
from replay_measured_turn import input_rows, run

BASE = Path(__file__).resolve().parent / "evidence"
LAGS = np.arange(-120, 121) / 1000
FILES = [BASE / "hw-rec-20261115/table-run1.jsonl", BASE / "hw-rec-20261118/run-e-spins.jsonl",
         BASE / "hw-rec-20261118/run-f-endurance-unit1.jsonl"]


def transition_losses(path):
    events = [json.loads(line) for line in path.read_text().splitlines()]
    states = [r for r in events if r["k"] == "s"]
    commands = [r for r in events if r["k"] == "c"]
    losses = []
    for i, cmd in enumerate(commands):
        if cmd.get("name") != "wheels" or cmd["left"] * cmd["right"] >= 0:
            continue
        stop = next(r for r in commands[i+1:] if r.get("name") == "stop")
        if stop.get("why") != "segment_end":
            continue
        duration = stop["t"] - cmd["t"]
        rows = [r for r in states if -.45 <= r["t"]-cmd["t"] <= duration+.75]
        t = np.array([r["t"]-cmd["t"] for r in rows])
        wheel = np.array([r["wr"]-r["wl"] for r in rows])
        gyro = np.array([r["gyro"][2] for r in rows])
        plateau = (t >= duration*.55) & (t <= duration*.85)
        # Normalize rest and plateau separately for each instrument, so a
        # constant yaw gain or surface difference cannot masquerade as delay.
        def normalize(y):
            zero = np.median(y[t < -.08])
            return (y-zero)/(np.median(y[plateau])-zero)
        wheel, gyro = normalize(wheel), normalize(gyro)
        edges = [(t >= 0) & (t < min(1.,duration*.55)),
                 (t >= duration-.12) & (t <= duration+.48)]
        scores = []
        for edge in edges:
            scores.append([np.mean((np.interp(t[edge]-lag,t,wheel)-gyro[edge])**2) for lag in LAGS])
        losses.append(scores)
    return np.array(losses)


def main():
    by_file = [transition_losses(path) for path in FILES]
    # Equal weight for each recording, then each turn, then each edge.
    pooled = np.mean([x.mean(axis=0) for x in by_file], axis=0)
    lag = LAGS[np.argmin(pooled.mean(axis=0))]
    for path, score in zip(FILES, by_file):
        print(path.name, len(score), "onset/brake (ms)", LAGS[score.mean(axis=0).argmin(axis=1)]*1000)
    print("pooled ms:", lag*1000, "separate edges:", LAGS[pooled.argmin(axis=1)]*1000)
    for i in range(3):
        keep = [s.mean(axis=(0,1)) for j,s in enumerate(by_file) if j != i]
        print("omit", FILES[i].name, "ms", LAGS[np.mean(keep,axis=0).argmin()]*1000)
    directory = BASE / "hw-rec-20261119"
    rows, t, wheels, obs, stop = input_rows(directory / "unit2-ccw100.jsonl")
    clock, out = run(rows,t,wheels,42,dense=True)
    steady = t[np.flatnonzero(np.min(abs(wheels),axis=1) >= 90)[0]]
    phase = np.select([t<0,t<steady,t<stop],["rest","spin-up","steady"],default="stopping")
    pre = np.interp(t,clock,out[:,0])
    post = np.interp(t-lag,clock,out[:,0])
    for label in ("spin-up","steady","stopping"):
        sel = phase == label
        print(label, "before/after mean and RMSE",
              [(round(float(y[sel].mean()),2),round(float(np.sqrt(np.mean((y[sel]-obs[sel])**2))),2))
               for y in (pre,post)])
    with (directory / "turn-lag-test.csv").open("w",newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s","phase","left_mmps","right_mmps","gyro_dps","before_dps","after_dps","lag_ms"])
        w.writerows(zip(t,phase,*wheels.T,obs,pre,post,np.full(len(t),1000*lag)))
    with (directory / "turn-lag-training.csv").open("w",newline="") as f:
        w = csv.writer(f)
        w.writerow(["lag_ms","recording","moves","onset_normalized_mse","brake_normalized_mse"])
        for path, score in zip(FILES,by_file):
            w.writerows((round(1000*l),path.stem,len(score),a,b)
                        for l,a,b in zip(LAGS,*score.mean(axis=0)))



def plot_results():
    import matplotlib.pyplot as plt
    directory = BASE / "hw-rec-20261119"
    series = np.genfromtxt(directory / "turn-lag-test.csv", delimiter=",", names=True, dtype=None, encoding="utf8")
    losses = np.genfromtxt(directory / "turn-lag-training.csv", delimiter=",", names=True, dtype=None, encoding="utf8")
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), gridspec_kw={"height_ratios":[1, 1.45, .8]})
    fig.subplots_adjust(top=.91, bottom=.08, hspace=.57, left=.11, right=.97)
    fig.suptitle("Does the gyro follow the wheel encoders?", x=.11, ha="left", weight="bold", fontsize=17)
    lag = series["lag_ms"][0]
    ax = axes[0]
    for path, label in zip(FILES, ["Nov 15 · 6 turns", "Nov 18 E · 4 turns", "Nov 18 F · 29 turns"]):
        part = losses[losses["recording"] == path.stem]
        ax.plot(part["lag_ms"], (part["onset_normalized_mse"]+part["brake_normalized_mse"])/2, label=label)
    ax.axvline(lag, color="#383640", ls=":", label=f"Frozen delay · {lag:.0f} ms")
    ax.set(xlabel="Gyro delay relative to encoders (ms)", ylabel="Normalized MSE",title="Calibration robot · transition error")
    ax.legend(ncol=2, fontsize=8)
    t, obs = series["time_s"], series["gyro_dps"]
    for ax in axes[1:]:
        ax.axvspan(0, min(t[series["phase"]=="steady"]), color="#f5dfbb", alpha=.35)
        ax.axvspan(min(t[series["phase"]=="steady"]), input_rows(directory/"unit2-ccw100.jsonl")[-1], color="#b9dfcc", alpha=.30)
        ax.set_xlim(-.12,2.22)
    axes[1].plot(t, obs, "o-", color="#202737", ms=3.3, label="Measured gyro")
    for key,color,style,label in [("before_dps","#ab613e","--","42 mm before"),("after_dps","#007a70","-","42 mm + frozen delay")]:
        axes[1].plot(t,series[key],style,color=color,label=label)
        axes[2].plot(t,series[key]-obs,style,color=color)
    axes[1].set(ylabel="Gyro Z (°/s)",title="Held-out robot · single 100 mm/s counterclockwise turn")
    axes[1].legend(ncol=3,fontsize=8,loc="lower center")
    axes[2].axhline(0,color=".45",lw=.7)
    axes[2].set(xlabel="Seconds from wheel-command issue",ylabel="Sim − robot (°/s)")
    for ax in axes:
        ax.grid(axis="y",alpha=.2)
        ax.spines[["top","right"]].set_visible(False)
    fig.savefig(BASE/"physics-plots"/"turn-lag-transfer.png",dpi=175)
    plt.close(fig)


if __name__ == "__main__":
    main()
    import sys
    if "--plot" in sys.argv:
        plot_results()
