"""Transfer docked cube acceleration spread to separate lift sessions."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    def strokes(folder):
        return [r for r in read(HERE.parent / folder / "moves.csv") if r["timing_valid"] == "1"]
    train = strokes("D1")
    # Median paired excess variance, preserving pre-command background per trial.
    def excess(rows):
        return np.median([float(r["moving_mg"])**2 - float(r["rest_mg"])**2 for r in rows])
    common = excess(train)
    directional = {d: excess([r for r in train if r["direction"] == d]) for d in ("raise", "lower")}
    cases = [("D2", r["direction"] + " " + r["repeat"], r["direction"],
              float(r["rest_mg"]), float(r["moving_mg"])) for r in strokes("D2")]
    for r in read(HERE.parent / "R11" / "lift-vibration.csv"):
        setting = "Full native raise" if r["run"] == "R11" else "Partial native raises"
        cases.append((setting, r["run"], "raise", float(r["rest_sd_mg"]), float(r["rise_sd_mg"])))
    out = []
    for setting, label, d, rest, measured in cases:
        out.append(dict(setting=setting, stroke=label, real_mg=measured, rest_mg=rest,
                        pooled_mg=np.sqrt(rest**2 + common),
                        directional_mg=np.sqrt(rest**2 + directional[d])))
    with (HERE / "vibration-transfer.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=out[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(out)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), sharey=True, layout="constrained")
    for i, ax in enumerate(axes):
        rows = [r for r in out if (r["setting"] == "D2") == (i == 0)]
        for key, marker, name in [("real_mg","o","Recorded"), ("rest_mg","_","Rest only"),
                                  ("pooled_mg","x","Pooled"), ("directional_mg","+","By direction")]:
            ax.plot(range(len(rows)), [r[key] for r in rows], marker, label=name, ms=8)
        ax.set(xticks=range(len(rows)), xticklabels=[r["stroke"] for r in rows],
               title=["Full strokes beside cube", "Native raises · only R11 is full height"][i])
        ax.tick_params(axis="x", labelrotation=30)
    axes[0].set_ylabel("Cube acceleration magnitude SD (mg)")
    fig.legend(*axes[0].get_legend_handles_labels(), loc="outside lower center", ncol=4)
    fig.savefig(HERE / "vibration-transfer.png", dpi=160)
    print("Added RMS amplitude by direction (mg)", {k:np.sqrt(x) for k,x in directional.items()})
    for name in dict.fromkeys(r["setting"] for r in out):
        group = [r for r in out if r["setting"] == name]
        print(name, [round(np.mean([abs(r[k]-r["real_mg"]) for r in group]),3)
                     for k in ("rest_mg","pooled_mg","directional_mg")])


if __name__ == "__main__":
    main()
