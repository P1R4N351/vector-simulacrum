"""Transfer a constant camera mount angle estimated only on the retreat."""
import csv

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize_scalar
from scipy.spatial.transform import Rotation

import reprojection as rp

K = rp.models["Unit 3"]
edges = np.genfromtxt(rp.here/"edges.csv", delimiter=",", names=True)
states = np.genfromtxt(rp.here.parent/"unit3-cube-contact-replay/inputs.csv", delimiter=",", names=True)
lag = 0.232  # Previous timing alignment on approaches 1 and 2.


def project(a, b, angle):
    # Rotate the optical frame around its center identically in both views.
    mount = Rotation.from_euler("x", angle, degrees=True).as_matrix()
    return rp.predict(a, b, K, mount)


_, a, b = rp.pairs[-1]
retreat = np.asarray(b["quad"])
result = minimize_scalar(lambda angle: np.mean((project(a,b,angle)-retreat)**2),
                         bounds=(-12,12), method="bounded")
print("Retreat mount delta", result.x)
fig, axes = plt.subplots(1,5,sharey=True,figsize=(12,3.5),layout="constrained")
rows = []
for i,ax in enumerate(axes,1):
    start = rp.on[(i-1)*2]
    s = states[states["trial"]==i]
    series = edges[(edges["trial"]==i) & (edges["seconds"]>1) & (edges["seconds"]<2.75)]
    for e in series:
        t = e["seconds"] + 1.5 - lag
        def state_at(key):
            return np.interp(t, s["sec"],s[key])
        end = {"state":dict(origin=start["state"]["origin"],xy=[state_at("x_mm"),state_at("y_mm")],
                             yaw=state_at("yaw_deg"),head=state_at("head_rad"))}
        actual = np.array([[e["u0"],e["v0"]],[e["u1"],e["v1"]]])
        for label,angle in [("Frozen",0),("Retreat fit",result.x)]:
            residual = project(start,end,angle)[:2] - actual
            for corner,(dx,dy) in enumerate(residual):
                rows.append((i,e["seconds"],label,corner,dx,dy))
    for label,style in [("Frozen","o-"),("Retreat fit","s--")]:
        r = np.array([[t,dx,dy] for trial,t,model,k,dx,dy in rows if trial==i and model==label])
        print(i,label,"vertical RMS",round(np.mean(r[:,2]**2)**.5,2),"2D RMS",round(np.mean(np.sum(r[:,1:]**2,axis=1))**.5,2))
        ax.plot(r[::2,0],r[:,2].reshape(-1,2).mean(axis=1),style,label=label)
    ax.axhline(0,color=".6",lw=1)
    ax.set(title=f"Approach {i}",xlabel="Time (s)")
axes[0].set_ylabel("Top edge vertical error (px)")
axes[-1].legend()
fig.savefig(rp.here/"mount-transfer.png",dpi=160)
with (rp.here/"mount-transfer.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["trial","seconds","model","corner","dx_px","dy_px"])
    w.writerows(rows)
