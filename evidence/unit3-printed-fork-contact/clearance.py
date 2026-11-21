"""Fork-to-cube clearances on the unmodified replay and hypothetical tilted poses."""
import csv
import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation
from evaluate import here, run


def closest(model, data):
    cube = data.body("cube")
    rc = cube.xmat.reshape(3,3)
    fork_ids = np.flatnonzero(model.geom_bodyid == model.body("fork").id)
    face_ids = np.flatnonzero(model.geom_bodyid == cube.id)
    best = {}
    segment = np.zeros(6)
    for c in face_ids:
        material = "rubber" if "rubber" in model.geom(c).name else "shell"
        for f in fork_ids:
            gap = mj.mj_geomDistance(model,data,c,f,.1,segment)
            if gap >= .1:
                continue
            point = (segment[:3] - cube.xpos) @ rc
            side = "right" if point[1] < 0 else "left"
            key = side, material
            if key not in best or gap < best[key][0]:
                best[key] = (gap,model.geom(c).name,model.geom(f).name,*point)
    return best


def main():
    rows = []
    def inspect(model,data,t,lift):
        if t < 4.749 or t > 6.251:
            return
        if abs(t*20-round(t*20)) > 1e-5:
            return
        for angle in ([0,20,35] if min(abs(t-5.5),abs(t-6))<.001 else [0]):
            copy = mj.MjData(model)
            copy.qpos[:] = data.qpos
            q = copy.joint("cube_root").qpos
            if angle:
                rot = Rotation.from_quat(q[3:],scalar_first=True)
                raised = rot * Rotation.from_euler("x",-angle,degrees=True)
                # Fix the left bottom edge midpoint in world coordinates.
                pivot = [0,.02215,-.02215]
                q[:3] += rot.apply(pivot) - raised.apply(pivot)
                q[3:] = raised.as_quat(scalar_first=True)
            mj.mj_forward(model,copy)
            for (side,mat),(gap,c,f,*point) in closest(model,copy).items():
                rows.append((t,lift,angle,side,mat,1000*gap,c,f,*(1000*np.asarray(point))))
    run((1,"stl","overhead","weld",0,0,0),inspect)
    with (here / "clearance.csv").open("w") as file:
        out = csv.writer(file,lineterminator="\n")
        out.writerow(["seconds","lift_mm","added_roll_right_up_deg","side","material","gap_mm",
                      "cube_geom","fork_geom","cube_local_x_mm","cube_local_y_mm","cube_local_z_mm"])
        out.writerows(rows)
    fig,(ax,force_ax)=plt.subplots(2,1,figsize=(8,6),sharex=True)
    for side in ("right","left"):
        for mat in ("rubber","shell"):
            q=[r for r in rows if r[2:5]==(0,side,mat)]
            ax.plot([r[0] for r in q],[r[5] for r in q],label=f"{side} {mat}",
                    linestyle="-" if side=="right" else "--")
    for t in (5.5,6.0):
        ax.axvline(t,color="gray",lw=.7)
    ax.set(xlabel="Seconds from approach",ylabel="Closest fork separation (mm)",
           title="Trial 1 · replayed cube pose")
    ax.legend()
    support = np.genfromtxt(here / "support.csv", delimiter=",", names=True)
    q = support[(support["trial"] == 1) & (support["sec"] >= 4.75) & (support["sec"] <= 6.25)]
    force_ax.plot(q["sec"],q["up_N"],label="Vertical force on cube")
    force_ax.axhline(0,color="gray",lw=.7)
    for t in (5.5,6.0):
        force_ax.axvline(t,color="gray",lw=.7)
    force_ax.set(xlabel="Seconds from approach", ylabel="Signed vertical force (N)")
    ax.set_xlabel("")
    fig.tight_layout()
    fig.savefig(here / "clearance.png",dpi=150)


if __name__=="__main__":
    main()
