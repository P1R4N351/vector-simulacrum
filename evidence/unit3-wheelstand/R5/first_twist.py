"""Locate yaw impulse during the approach in the free-cube replay."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import read

here = Path(__file__).resolve().parent
glyph = read(here/"approach-glyph-axes.csv")
glyph = glyph[(glyph["width_mm"] == 23) & (glyph["sec"]>1.95)]
end = glyph["sec"][glyph["frame"]==81].item()
gate = read(here/"pickup-onset.csv")
onset = gate[(gate["pickup"]=="R5") & (gate["gate_g"]==.05)]["first_packet_s"].item()
fig, axes = plt.subplots(2,1,figsize=(8,6),layout="constrained",sharex=True)
axes[0].plot(glyph["sec"],glyph["yaw_change_deg"],"ko-",label="Shelf glyph")
rows = []
for path in map(Path,sys.argv[1:]):
    a = read(path/"world-moment.csv")
    dt = np.median(np.diff(a["s"]))
    use = a["s"] <= glyph["sec"][-1]
    t = a["s"][use]
    torque = np.column_stack([a[k][use] for k in ("shelf_z","fork_z","link_z")])
    impulse = np.vstack([np.zeros(3),np.cumsum(torque[:-1]*dt,axis=0)])
    np.testing.assert_allclose(impulse.sum(axis=1),1.6133333333333334e-5*(a["wz"][use]-a["wz"][0]),atol=1e-11)
    heading = read(path/"cube-heading.csv")
    angle = np.rad2deg(np.unwrap(heading["yaw_rad"])-heading["yaw_rad"][0])
    window=(heading["sec"]>=t[0]-.02)&(heading["sec"]<=t[-1]+.005)
    label = f"{1000*dt:.3g} ms"
    axes[0].plot(heading["sec"][window],angle[window],label=label)
    for i,color in enumerate(("C0","C1")):
        axes[1].plot(t,1000*impulse[:,i],color=color,ls="-" if dt>.0002 else "--",
                     label=f"{('Shelf','Fork')[i]} · {label}")
    hits = read(path/"approach-yaw-contacts.csv")
    hits = hits[hits["sec"]<end]
    normal = hits["normal_yaw_Nm"]
    other = hits["other_yaw_Nm"]
    impulse_fork = (normal+other).sum()*dt
    mask = a["s"]<end
    np.testing.assert_allclose(impulse_fork,dt*a["fork_z"][mask].sum(),atol=1e-10)
    for side in ("positive","negative"):
        for rubber in (False,True):
            sel = ((hits["fork_y_m"] >= 0) == (side=="positive")) & (np.char.find(hits["cube_geom"],"rubber")>=0 if rubber else np.char.find(hits["cube_geom"],"rubber")<0)
            w=hits["normal_N"][sel]
            rows.append([1000*dt,side,"rubber" if rubber else "shell",hits["sec"][sel].min() if len(w) else "",
                         w.sum()*dt, 1000*np.average(hits["fork_y_m"][sel],weights=w) if len(w) else "",
                         normal[sel].sum()*dt*1000,other[sel].sum()*dt*1000])
for ax in axes:
    ax.axvline(onset,color=".4",ls=":",lw=1,label="Cube accelerometer onset")
    ax.legend(fontsize=7,ncol=3)
axes[0].set_ylabel("Yaw change (°)")
axes[1].set(xlabel="Seconds after pickup command",ylabel="Accumulated yaw impulse (mN·m·s)")
fig.savefig(here/"first-twist.png",dpi=160)
with (here/"first-twist.csv").open("w") as stream:
    writer=csv.writer(stream,lineterminator="\n")
    writer.writerow("step_ms fork_y_side material first_s normal_Ns weighted_fork_y_mm normal_yaw_mNms other_yaw_mNms".split())
    writer.writerows(rows)
