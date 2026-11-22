"""Plot the two integration steps at each solver setting, with collider order paired."""
import csv
from itertools import product
from pathlib import Path

import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent


def main():
    groups = [("A", "4"), ("B", "4"), ("B", "1")]
    fig, axes = plt.subplots(2, 3, sharex="col", sharey="row", figsize=(11, 6), layout="constrained")
    for idx, (precision, order) in enumerate(product(("default","tight"),("original","reverse"))):
        with (HERE/f"solver-{precision}-{order}.csv").open() as f:
            data = list(csv.DictReader(f))
        for col, (view, layout) in enumerate(groups):
            subset = sorted((r for r in data if (r["view"],r["layout"])==(view,layout)),
                            key=lambda r: -float(r["dt_ms"]))
            assert len(subset)==(3 if precision=="tight" and (view,layout)==("B","4") else 2)
            for row, key in enumerate(("travel_mm","ccw_deg")):
                ax = axes[row,col]
                x = [i+(idx-1.5)*.024 for i in range(len(subset))]
                ax.plot(x,[float(r[key]) for r in subset],color=("tab:blue","tab:orange")[idx//2],
                        linestyle="--" if idx%2 else "-",marker="x" if idx%2 else "o",
                        label=f"{precision}, {order}")
    for col,(view,layout) in enumerate(groups):
        axes[0,col].set_title(f"View {view}, orientation {layout}")
        for row, band in enumerate(((39.8,42.9),(64.8,70.9))):
            axes[row,col].axhspan(*band,color="seagreen",alpha=.25,label="Physical estimate")
        ticks = ["0.250","0.125","0.0625"] if (view,layout)==("B","4") else ["0.250","0.125"]
        axes[1,col].set_xticks(range(len(ticks)),ticks)
    axes[1,0].set_ylabel("Yaw (degrees CCW)")
    axes[0,0].set_ylabel("Displacement (mm)")
    axes[0,2].legend(fontsize=8)
    fig.supxlabel("Integration step (ms)")
    fig.savefig(HERE/"solver-check.png",dpi=150)


if __name__=="__main__":
    main()
