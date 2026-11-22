"""Let the fork yield along the gravity axis, holding chassis pitch to W20."""
import csv
import xml.etree.ElementTree as ET
from yielding_drive import Drive
from free_departure import here, mj, np, plt, poses, replay

class ForkDrive(Drive):
    def __init__(self, force, control):
        super().__init__(1.5, control)
        self.fork_force = force

    def build(self, root, forward):
        super().build(root, forward)
        carrier = root.find(".//body[@name='carrier']")
        carrier.find("inertial").set("mass", ".164")
        fork = carrier.find("body[@name='fork']")
        carrier.remove(fork)
        slide = ET.SubElement(carrier, "body", name="vertical_carriage")
        ET.SubElement(slide, "joint", name="vertical", type="slide", axis="0 0 1")
        ET.SubElement(slide, "inertial", pos="0 0 0", mass=".006", diaginertia=".000001 .000001 .000001")
        slide.append(fork)

    def step(self, m, d, base, fork, quat, i, dt):
        super().step(m, d, base, fork, quat, i, dt)
        m.body("fork").pos[2] = 0
        target = fork[2]
        lift = d.joint("vertical")
        if i == 0:
            lift.qpos[0] = target
            self.last_z = target
        speed = (target-self.last_z)/dt
        self.last_z = target
        self.z_error = target-lift.qpos[0]
        force = .006*9.81+180*self.z_error+2*(speed-lift.qvel[0])
        lift.qfrc_applied[0] = np.clip(force,-self.fork_force,self.fork_force)

fig, axes = plt.subplots(3,2,figsize=(10,7),sharex=True,layout="constrained")
with (here/"vertical-fork.csv").open("w") as out:
    writer = csv.writer(out,lineterminator="\n")
    writer.writerow("pose control fork_cap_N dt_ms time_s target_height_mm actual_height_mm center_height_mm shelf_load_N chassis_lag_mm".split())
    for j,pose in enumerate(poses[poses["run"]=="W20"][-2:]):
        for control in (True,False):
            for cap in (.3,1.0):
                for dt in (.00025,.000125):
                    drive = ForkDrive(cap,control)
                    rows = []
                    def observe(t,m,d,edge):
                        load = 0.
                        for i,c in enumerate(d.contact):
                            if m.geom("table").id not in c.geom:
                                continue
                            w = np.zeros(6)
                            mj.mj_contactForce(m,d,i,w)
                            load += (1 if c.geom[0]==m.geom("table").id else -1)*(c.frame.reshape(3,3).T@w[:3])[2]
                        row = (t,1000*drive.last_z,1000*d.joint("vertical").qpos[0],1000*d.qpos[2],load,1000*drive.error)
                        rows.append(row)
                        writer.writerow((j+1,control,cap,dt*1000,*row))
                    replay(pose,0,dt,("body","wheels"),observe=observe,drive=drive)
                    a = np.array(rows)
                    if not control and dt==.000125:
                        for ax,k in zip(axes[:,j],(2,4,3)):
                            ax.plot(a[:,0],a[:,k],label=f"{cap:g} N")
                        if cap==.3:
                            axes[0,j].plot(a[:,0],a[:,1],"k--",label="Encoder-derived height")
for ax,title in zip(axes[0],("Earlier cube sighting","Final cube sighting")):
    ax.set_title(title)
    ax.legend(fontsize=8)
for ax,label in zip(axes[:,0],("Fork height (mm)","Shelf force (N)","Cube center height (mm)")):
    ax.set_ylabel(label)
for ax in axes[-1]:
    ax.set_xlabel("Seconds from command")
fig.savefig(here/"vertical-fork.png",dpi=150)
