"""Compare measured and predicted motion and check collider-order sensitivity."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = Path(__file__).resolve().parent


def endpoints(name):
    with (HERE / name).open() as stream:
        return {(r["view"], r["layout"]): (float(r["travel_mm"]), float(r["ccw_deg"]))
                for r in csv.DictReader(stream) if float(r["dt_ms"]) == .25}


def main():
    standard = endpoints("face-replay.csv")
    reversed_order = endpoints("face-replay-reverse.csv")
    fig, axes = plt.subplots(1, 2, figsize=(11, 5), sharex=True, sharey=True, layout="constrained")
    for view, ax in zip("AB", axes):
        ax.add_patch(Rectangle((39.8,64.8), 3.1, 6.1, fc="#41874a", label="Physical endpoints"))
        for i, layout in enumerate(("upright","1","2","3","4")):
            pair = [standard[view,layout],reversed_order[view,layout]]
            color = f"C{i}"
            ax.plot(*zip(*pair),color=color,alpha=.55,linewidth=1)
            ax.scatter(*pair[0],color=color,label=layout)
            ax.scatter(*pair[1],color=color,marker="x")
        ax.axhline(0,color="0.7",lw=.8)
        ax.set_title(f"Camera {view} registration")
        ax.set_xlabel("Final displacement (mm)")
    axes[0].set_ylabel("Counterclockwise rotation (degrees)")
    axes[1].legend(title="Starting orientation",fontsize=8,loc="lower right")
    fig.supxlabel("Dots use the original box order. Crosses reverse it.",fontsize=10)
    fig.savefig(HERE/"face-endpoints.png",dpi=150)


if __name__=="__main__":
    main()
