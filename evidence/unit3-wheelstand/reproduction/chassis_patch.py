"""Locate simulated box contacts against the registered stock animation-rig shell."""
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np

import replay
from simulacrum import body_mesh

HERE = Path(__file__).resolve().parent


def shell_triangles(path, values):
    if body_mesh.sha256_file(path) != body_mesh.RIG["sha256"]:
        raise ValueError("Unexpected rig digest")
    verts, groups = body_mesh.read_obj(path)
    mm = body_mesh.to_robot(verts, body_mesh.register(verts, groups))
    feats = body_mesh.measure(mm, groups)
    errors = body_mesh.check_against_table(values, feats)
    if errors:
        raise ValueError(errors)
    parts = body_mesh.split_parts(mm, groups, body_mesh.part_frames(values, feats))
    points, tris = body_mesh.merge(parts["chassis"]["collide"])
    return points[tris]


def foremost(tris, y, z):
    # Intersect a fore-aft line with the stock shell at the contact's lateral position and height.
    a, b, c = tris.transpose(1, 0, 2)
    u, v = b-a, c-a
    det = u[:,1]*v[:,2] - u[:,2]*v[:,1]
    valid = abs(det) > 1e-10
    u, v, a, det = u[valid], v[valid], a[valid], det[valid]
    dy, dz = y-a[:,1], z-a[:,2]
    p = (dy*v[:,2]-dz*v[:,1])/det
    q = (u[:,1]*dz-u[:,2]*dy)/det
    hit = (p>=-1e-9)&(q>=-1e-9)&(p+q<=1+1e-9)
    return max((a[:,0]+p*u[:,0]+q*v[:,0])[hit], default=np.nan)


def main(fixture, rig):
    v = replay.params.values(replay.params.load(model="vector_2"), {"tip_supports_driven":1})
    tris = shell_triangles(Path(rig), v)
    m = replay.build(v, fixture)
    m.opt.timestep = v["sim_timestep_ms"]*.001/4
    box = m.geom("chassis_geom").id
    fig, axes = plt.subplots(1, 2, figsize=(10,4), layout="constrained", sharey=True)
    with (HERE/"chassis-patch.csv").open("w", newline="") as f:
        out = csv.writer(f, lineterminator="\n")
        out.writerow(["trial","case","tap_s","peak_single_box_contact_N","x_mm","y_mm","z_mm","rig_front_mm","box_point_minus_rig_front_mm"])
        for name, ax in zip(("W4","W8"), axes):
            tr = replay.Trial(name)
            packets = tr.cube[::3]
            t = packets["sec"]-tr.zero
            end = min(t[-1],tr.late+1.2)
            changes = t[1:][np.diff(packets["tap"])!=0]
            last = max(changes[(changes>=tr.late-.3)&(changes<=end)])
            for case,(s,o) in enumerate(((s,o) for s in (3.5,3.8) for o in (tr.early,tr.late)),1):
                contacts, events = [], []
                wrench = np.empty(6)
                def step(r, t):
                    if not last+.03 <= t <= end:
                        return
                    for i,c in enumerate(r.d.contact):
                        if box in c.geom and r.cube_id in m.geom_bodyid[c.geom]:
                            mujoco.mj_contactForce(m,r.d,i,wrench)
                            if wrench[0]>0:
                                pos = r.d.xmat[r.chassis].reshape(3,3).T @ (c.pos-r.d.xpos[r.chassis])
                                contacts.append((t,wrench[0],pos*1000))
                replay.run(m,v,tr,s+tr.shift,o,"servo:60",17,tap_times=events,on_step=step)
                for event in (e for e in events if last+.06<e<=end):
                    win = [p for p in contacts if event-.03<p[0]<=event+1e-8]
                    if not win:
                        out.writerow([name,case,event,0,"","","","",""])
                        continue
                    _, force, (x,y,z) = max(win,key=lambda p:p[1])
                    front = foremost(tris,y,z)
                    out.writerow([name,case,event,force,x,y,z,front,x-front])
                    if np.isfinite(front):
                        ax.plot([front,x],[z,z],color=".6")
                        ax.scatter(front,z,marker="|",s=120,color="tab:blue")
                    ax.scatter(x,z,s=30,color="tab:red",marker="o" if np.isfinite(front) else "x")
            ax.set_title(name+" · late tap windows")
            ax.set_xlabel("Forward position in chassis frame (mm)")
            ax.grid(alpha=.2)
    axes[0].set_ylabel("Height in chassis frame (mm)")
    fig.suptitle("Box contact (red) and rig front (blue) · crosses mark lines missing the shell")
    fig.savefig(HERE/"chassis-patch.png",dpi=160)


if __name__ == "__main__":
    main(*sys.argv[1:])
