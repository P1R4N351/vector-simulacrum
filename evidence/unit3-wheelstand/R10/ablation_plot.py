"""Plot contact-pair ablations with the physical endpoint estimates."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def plot(folder):
    with (folder/'contact-ablation.csv').open() as f:
        rows = [r for r in csv.DictReader(f) if float(r['dt_ms']) == .25]
    fig, axes = plt.subplots(1, 2, figsize=(10,4.5), layout='constrained')
    for ax, column, limits, label in zip(axes, ['travel_mm','ccw_deg'], [(39.8,42.9),(64.8,70.9)],
                                          ['Displacement (mm)','Rotation (° CCW)']):
        ax.axhspan(*limits, color='#78ac8b', alpha=.55, label='Observed endpoints')
        for i,view in enumerate('AB'):
            values = [float(r[column]) for r in rows if r['view'] == view]
            ax.bar(np.arange(5)+(i-.5)*.34,values,.33,label='Start from camera '+view)
        ax.axhline(0,color='.4',lw=.7)
        ax.set(xticks=range(5),xticklabels=['All','No fork','No tread','No shell','None'],ylabel=label)
    axes[0].legend(fontsize=8)
    fig.savefig(folder/'contact-ablation.png', dpi=150)
    plt.close(fig)


if __name__=='__main__':
    plot(Path(__file__).resolve().parent)
