"""Fit the observed raw-word lattice on rest, then score untouched captures."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from decode import decode
from stationary_noise import observations, scatter, simulate, v

here = Path(__file__).resolve().parent
names = ("resting", "undock-check", "pickup")
training = np.rint(observations("resting") * 8192).astype(int)
codes = np.unique(training)
spacing = int(np.gcd.reduce(np.diff(codes)))
tag = int(codes[0] % spacing)
assert spacing == 1 << (spacing.bit_length()-1)
unit_format = [spacing.bit_length()-1, tag]
assert unit_format == v["cube_accel_word_format"]
force = training.mean(axis=(0,1))/8192
rows = []
fig, axes = plt.subplots(1,3,figsize=(10,3.8),sharey=True,layout="constrained")
for i, (name, ax) in enumerate(zip(names,axes)):
    packets,_ = decode(here/(name+".tsv"))
    raw=np.stack([np.frombuffer(p[3],dtype="<i2") for p in packets]).ravel()
    static=observations(name)
    fractions=np.bincount(raw%16,minlength=16)/raw.size
    ax.bar(np.arange(16)-.25,fractions,.25,label="Recorded")
    for j,(label,fmt) in enumerate([("Original",[4,0]),("Unit 3",unit_format)]):
        sim=simulate(v["cube_accel_noise_density_ug"],force,10+i,fmt)
        codes=np.rint(sim*8192).astype(int)
        freq=np.bincount(codes.ravel()%16,minlength=16)/codes.size
        ax.bar(np.arange(16)+j*.25,freq,.25,label=label)
        rows.append([name,label,len(raw),float(np.mean(raw%(1<<fmt[0])==fmt[1])),
                     round(.5*np.abs(fractions-freq).sum(),4),
                     round(np.sqrt(np.mean((scatter(static)-scatter(sim))**2)),3),
                     round(1000*np.linalg.norm(static.mean(axis=(0,1))),3)])
    ax.set(title=name,xlabel="Raw word modulo 16",xticks=[0,1,5,9,13])
axes[0].set_ylabel("Fraction of axis words")
axes[-1].legend(fontsize=8)
fig.savefig(here/"word-format.png",dpi=150)
with (here/"word-format.csv").open("w") as f:
    writer=csv.writer(f,lineterminator="\n")
    writer.writerow(["capture","format","axis_words","observed_format_coverage","residue_TV_distance",
                     "stationary_scatter_rmse_mg","stationary_mean_norm_mg"])
    writer.writerows(rows)
