"""Trace support on the cube during the measured W20 return-to-dock motion."""
import csv
from free_departure import here, mj, np, plt, poses, replay

motion = np.genfromtxt(here/"retreat-input.csv", names=True, delimiter=",")
motion["yaw"] = np.unwrap(motion["yaw"])
fig, axes = plt.subplots(2, 2, figsize=(10, 6), sharex=True, layout="constrained")
with (here/"release-support.csv").open("w") as output:
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow("pose dt_ms time_s past_edge_mm center_z_mm table_N fork_N chassis_N wheel_N".split())
    for col, (name, pose) in enumerate(zip(("Earlier", "Final"), poses[poses["run"]=="W20"][-2:])):
        for dt in (.00025, .000125):
            samples = []
            def observe(sec, m, d, past):
                loads = np.zeros(4)
                for contact_id, c in enumerate(d.contact):
                    body = [m.body(m.geom_bodyid[g]).name for g in c.geom]
                    if "cube" not in body:
                        continue
                    other = 1-body.index("cube")
                    geom = m.geom(c.geom[other]).name
                    group = 0 if geom=="table" else 1 if body[other]=="fork" else 2 if geom=="chassis_geom" else 3
                    wrench = np.zeros(6)
                    mj.mj_contactForce(m, d, contact_id, wrench)
                    loads[group] += (1 if other==0 else -1)*(c.frame.reshape(3,3).T @ wrench[:3])[2]
                row = [sec, past, d.qpos[2]*1000, *loads]
                samples.append(row)
                writer.writerow([name, dt*1000, *row])
            trace = replay(pose, 0, dt, ("body","wheels"), motion, 8.6, observe)
            below = trace[trace[:,3]<0]
            print(name, dt, "below", below[0,0] if len(below) else None, "last", trace[-1])
            if dt==.000125:
                a = np.array(samples)
                axes[0,col].plot(a[:,0], a[:,1], label="Center past edge")
                axes[0,col].plot(a[:,0], a[:,2], label="Center above shelf")
                for i, label in enumerate(("Shelf","Fork","Chassis","Wheels")):
                    axes[1,col].plot(a[:,0], a[:,3+i], label=label)
                axes[0,col].set_title(name+" cube sighting")
                axes[1,col].set_xlabel("Seconds from command")
for ax in axes.flat:
    ax.axhline(0, color="gray", lw=.6)
    ax.legend(fontsize=8)
axes[0,0].set_ylabel("Distance (mm)")
axes[1,0].set_ylabel("Vertical contact force (N)")
fig.savefig(here/"release-support.png", dpi=150)
