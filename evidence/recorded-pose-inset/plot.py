"""Draw overlays directly from the scored spin replays."""
from pathlib import Path
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
COLORS = ["#202c39", "#a35b20", "#167484"]
NAMES = ["Robot", "Original axles", "Fitted contact"]
plt.rcParams.update({"font.size": 10, "axes.spines.right": False, "axes.spines.top": False,
                     "figure.facecolor": "white", "savefig.facecolor": "white"})


def read(name):
    with (HERE / name).open() as stream:
        return list(csv.DictReader(stream))


def draw(dataset, traces, metrics, sources, fields, series_key, interval):
    spins = list(dict.fromkeys(r["spin"] for r in traces))
    fig, axes = plt.subplots(3, len(spins), figsize=(14, 7), sharex="col", sharey="row",
                             layout="constrained")
    for col, spin in enumerate(spins):
        axcol = axes[:, col]
        a, b = interval(spin, metrics)
        for ax in axcol:
            ax.axvspan(a, b, color="#b8c8bc", alpha=.23, linewidth=0)
            ax.axhline(0, color="#c5c5c5", lw=.6)
        for j, source in enumerate(sources):
            rows = [r for r in traces if r["spin"] == spin and r[series_key] == source]
            assert rows, (spin,source)
            for ax, key in zip(axcol, fields):
                ax.plot([float(r["t_s"]) for r in rows], [float(r[key]) for r in rows],
                        color=COLORS[j], lw=1.45 if j == 0 else 1.1,
                        ls=["-", "--", "-"][j], marker="." if j == 0 else None, ms=3,
                        alpha=.93, label=NAMES[j], zorder=4 if j == 0 else 3)
        axcol[0].set_title(spin.replace("_"," ").replace("-"," "))
        axcol[-1].set_xlabel("Time from command (s)")
    for axis, name in zip(axes[:, 0], ["Yaw", "Roll", "Pitch"]):
        axis.set_ylabel(name + " (rad/s)")
    axes[0,0].legend(loc="best", fontsize=8)
    fig.suptitle(f"{dataset}   |   Gyro through each spin", fontsize=17, weight="bold")
    fig.supxlabel("Shading marks the steady interval used for scoring. Dots are recorded samples.", fontsize=10)
    fig.savefig(HERE / (dataset.lower()+"-spins.png"), dpi=150)
    plt.close(fig)


def compare_fit(p2, c7):
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8), layout="constrained")
    ax = axes[0,0]
    grid = [r for r in read("grid.csv") if float(r["inset_mm"]) >= 0]
    ax.plot([float(r["inset_mm"]) for r in grid], [100*float(r["rms_fractional_yaw_error"]) for r in grid],
            color=COLORS[2], marker=".", lw=1.3)
    ax.axvline(3, ls=":", color="#777", label="Previous 42 mm span")
    ax.axvline(3.75, ls="--", color=COLORS[2], label="Selected 40.5 mm span")
    ax.set(xlabel="Longitudinal inset per end (mm)", ylabel="RMS relative yaw error (%)", title="unit 3 fit across four spins")
    ax.legend(fontsize=8)
    order = list(dict.fromkeys(r["spin"] for r in p2))
    ax = axes[0,1]
    for key, label, color, marker in zip(["recorded","stock","proxy_3","proxy_3.75"],
                                         ["Camera","Original axles","42 mm span","40.5 mm span"],
                                         [COLORS[0],COLORS[1],"#818997",COLORS[2]],["o","x","^","s"]):
        ys=[float(next(r for r in p2 if r["spin"]==s and r["series"]==key)["center_mm"]) for s in order]
        ax.plot(range(4),ys, marker=marker, lw=1, color=color,label=label)
    ax.set_xticks(range(4),[s.replace("_"," ").upper() for s in order])
    ax.set(ylabel="Turn center x from front axle (mm)", title="unit 3 camera and simulated pivot")
    ax.legend(fontsize=8,ncol=2)
    for ax, rows, ids, key, roll, pitch, title in [
        (axes[1,0],p2,["recorded","stock","proxy_0","proxy_3.75"],"series","roll","pitch","unit 3 at the recorded pose"),
        (axes[1,1],c7,["unit 1","original","proxy_zero","frozen"],"source","mean_abs_roll_rad_s","mean_abs_pitch_rad_s","unit 1 held-out spins")]:
        for i,(field,color) in enumerate([(roll,"#4d528b"),(pitch,"#408271")]):
            vals = [np.mean([float(r[field]) for r in rows if r[key]==s]) for s in ids]
            ax.bar(np.arange(4)+(i-.5)*.32, vals, .32, color=color,label=["Roll","Pitch"][i])
        ax.set_xticks(range(4),["Robot","Original","Zero inset\nproxy","3.75 mm\ninset"])
        ax.set(ylabel="Mean absolute gyro rate (rad/s)", title=title)
        ax.legend(fontsize=8)
        ax.set_ylim(0,.7)
    fig.suptitle("Contact spacing fits yaw better than pivot or rocking", fontsize=17, weight="bold")
    fig.savefig(HERE/"fit-and-residuals.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    p2, c7 = read("metrics.csv"), read("c7_scores.csv")
    draw("unit 3", read("traces.csv"), p2, ["recorded","stock","proxy_3.75"], ["yaw","roll","pitch"], "series",lambda s,m:(.7,1.4))
    def c7_window(spin, metrics):
        row=next(r for r in metrics if r["spin"]==spin)
        return float(row["window_start_s"]),float(row["window_end_s"])
    draw("unit 1", read("c7_traces.csv"), c7, ["unit 1","original","frozen"],
         ["yaw_rad_s","roll_rad_s","pitch_rad_s"], "source",c7_window)
    compare_fit(p2,c7)
