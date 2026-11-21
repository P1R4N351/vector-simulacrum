"""Signed height differences across the 23 mm top glyph in trial 1."""
import csv
import cv2
import mujoco
import matplotlib.pyplot as plt
import numpy as np
from evaluate import run, here
from tilt import p, square

rows = p[p["trial"] == 1]
qs = np.array([list(row)[2:] for row in rows]).reshape(-1,4,2)


def optical(k, focal):
    principal = np.array([912.9463,541.1996])
    intrinsics = np.array([[focal,0,principal[0]], [0,focal,principal[1]], [0,0,1.]])
    def rotation(q):
        d = q + [420,485] - principal
        uv = principal + d / (1 + k*np.sum((d/960)**2,axis=1)[:,None])
        ok, r, _ = cv2.solvePnP(square, uv, intrinsics, None)
        if not ok:
            raise ValueError("Top glyph pose failed")
        return cv2.Rodrigues(r)[0]
    up = -rotation(np.median(qs[rows["sec"]<0],axis=0))[:,2]
    # Glyph +x runs image-right (shelf-left); +y runs image-down (shelf-forward).
    return np.array([[t, *(23 * (up @ rotation(q))[ [1,0] ])]
                     for t,q in zip(rows["sec"],qs)])


def main():
    trace = []
    support = []
    def observer(model,data,t,lift):
        R = data.body("cube").xmat.reshape(3,3)
        trace.append((t,23*R[2,0],23*R[2,1]))
        cube = data.body("cube")
        fork_id = model.body("fork").id
        sides = [0., 0.]
        for i, contact in enumerate(data.contact):
            first, second = model.geom_bodyid[contact.geom]
            if {first, second} != {fork_id, cube.id}:
                continue
            wrench = np.empty(6)
            mujoco.mj_contactForce(model, data, i, wrench)
            force = (1 if second == cube.id else -1) * wrench[:3] @ contact.frame.reshape(3,3)
            left = (contact.pos - cube.xpos) @ R[:,1] > 0
            sides[int(left)] += max(0., force[2])
        support.append((t,*sides))
    run((1,"stl","overhead","weld",0,0,0), observer, observer_stride=1)
    contact = np.asarray(support)
    with (here / "tip-support.csv").open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["from_sec","to_sec","right_upward_impulse_Ns","left_upward_impulse_Ns"])
        for start, end in ((5,5.5),(5.5,6),(6,6.5)):
            take = (contact[:,0] >= start-1e-8) & (contact[:,0] < end-1e-8)
            writer.writerow([start,end,*contact[take,1:].sum(axis=0)*.001])
    curves = {"Printed-fork replay":np.array(trace)[::5], "See3CAM calibration":optical(-1.499,473.11),
              "Pinhole sensitivity":optical(0,700)}
    fig, axes = plt.subplots(2,1,sharex=True,figsize=(8,5))
    with (here / "tip-direction.csv").open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["source","approach_sec","forward_edge_minus_rear_mm","left_edge_minus_right_mm"])
        for label,curve in curves.items():
            for t,forward,left in curve:
                writer.writerow([label,t,forward,left])
            for j,ax in enumerate(axes,1):
                ax.plot(curve[:,0],curve[:,j],"-" if label.startswith("Printed") else ".-",label=label)
    for ax,title in zip(axes,("Forward edge minus rear edge","Left edge minus right edge")):
        ax.set(title=title,ylabel="Height difference (mm)",xlim=(3.5,7.3))
        ax.axhline(0,color="grey",lw=.6)
    axes[0].legend(fontsize=8)
    axes[1].set_xlabel("Seconds from approach")
    fig.tight_layout()
    fig.savefig(here / "tip-direction.png",dpi=160)
    for label,curve in curves.items():
        print(label,curve[np.argmin(abs(curve[:,0]-5.498))],curve[np.argmin(abs(curve[:,0]-5.998))])

if __name__ == "__main__":
    main()
