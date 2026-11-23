"""Check within-packet variation and high excursions on withheld shelf recordings."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from axis_noise import quiet_packets
from stationary_noise import observations, simulate, v

here = Path(__file__).resolve().parent
rest = observations("resting")
force = rest.mean(axis=(0, 1))
scale = np.sqrt(np.mean(np.diff(rest, axis=1)**2, axis=(0, 1)))
model = np.concatenate([simulate(v["cube_accel_noise_density_ug"], force, seed,
                                 v["cube_accel_word_format"], v["cube_accel_noise_axis_scale"])
                        for seed in range(8)])

def excursions(packets):
    return np.linalg.norm(np.diff(packets, axis=1)/scale, axis=2).max(axis=1)

threshold = np.quantile(excursions(rest), .99)
fig, axes = plt.subplots(1, 2, figsize=(9, 3.8), layout="constrained")
with (here/"packet-noise-transfer.csv").open("w", newline="") as handle:
    w = csv.writer(handle)
    w.writerow(["session","packets","median","p95","p99","threshold","above_threshold",
                "corr_xy","corr_xz","corr_yz","two_slot_ratio_x","two_slot_ratio_y","two_slot_ratio_z"])
    for name, packets in [("Rest", rest), ("Simulator", model), ("D1", quiet_packets("D1")),
                          ("D2", quiet_packets("D2")), ("Undock check", observations("undock-check"))]:
        values = excursions(packets)
        delta = np.diff(packets, axis=1).reshape(-1,3)
        corr = np.corrcoef(delta.T)[np.triu_indices(3,1)]
        ratio = np.sqrt(np.mean((packets[:,0]-packets[:,2])**2,axis=0)/np.mean(delta**2,axis=0))
        w.writerow([name,len(values),*np.quantile(values,[.5,.95,.99]),threshold,
                    np.sum(values>threshold),*corr,*ratio])
        line, = axes[0].plot(np.sort(values),np.arange(len(values),0,-1)/len(values),label=name)
        axes[1].plot(['X','Y','Z'],ratio,'o-',color=line.get_color(),label=name)
axes[0].axvline(threshold,color='gray',ls='--')
axes[0].set(xlabel='Largest standardized XYZ step per packet',ylabel='Fraction at or above',yscale='log')
axes[1].set(xlabel='Cube axis',ylabel='Two-slot / one-slot RMS step')
axes[1].legend(fontsize=8)
fig.savefig(here/"packet-noise-transfer.png",dpi=160)
