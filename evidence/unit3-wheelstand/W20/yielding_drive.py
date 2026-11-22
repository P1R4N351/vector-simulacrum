"""W20 sensitivity to finite longitudinal drive authority at the measured posture."""
import csv
import sys
import xml.etree.ElementTree as ET
from free_departure import here, mj, np, plt, poses, replay

control = "--no-contact" in sys.argv[2:]
output_name = "yielding-free-control" if control else "yielding-drive"

class Drive:
    def __init__(self, force, free_control=control):
        self.free_control = free_control
        self.force = force

    def build(self, root, forward):
        self.axis = np.r_[forward, 0.]
        world = root.find("worldbody")
        world.find("geom[@name='table']").set("conaffinity", "4")
        carrier = ET.SubElement(world, "body", name="carrier")
        ET.SubElement(carrier, "joint", name="travel", type="slide", axis=" ".join(map(str,self.axis)))
        # All longitudinal inertia belongs to the carrier, including the fork.
        ET.SubElement(carrier, "inertial", pos="0 0 0", mass=".17", diaginertia=".0001 .0001 .0001")
        for name in ("fork","front_contact"):
            body = world.find(f"body[@name='{name}']")
            world.remove(body)
            body.attrib.pop("mocap")
            carrier.append(body)
            for geom in body.iter("geom"):
                geom.set("mass","0")
                if self.free_control:
                    geom.set("contype","0")
                    geom.set("conaffinity","0")

    def step(self, m, d, base, fork, quat, i, dt):
        travel = d.joint("travel")
        target = base @ self.axis
        if i == 0:
            travel.qpos[0] = target
            self.previous = target
        v = (target-self.previous)/dt
        self.previous = target
        self.error = target-travel.qpos[0]
        travel.qfrc_applied[0] = np.clip(120*self.error+8*(v-travel.qvel[0]),-self.force,self.force)
        for name, position in (("fork",fork),("front_contact",base)):
            body = m.body(name)
            body.pos[:] = position-target*self.axis
            body.quat[:] = quat

if __name__ == "__main__":
    fig, axes = plt.subplots(3,2,figsize=(10,7),sharex=True,layout="constrained")
    with (here/(output_name+".csv")).open("w") as f:
        writer = csv.writer(f,lineterminator="\n")
        writer.writerow("pose cap_N step_ms time_s center_past_edge_mm center_z_mm longitudinal_error_mm shelf_vertical_N".split())
        for j,pose in enumerate(poses[poses["run"]=="W20"][-2:]):
            for cap in (.5,1.5):
                for dt in (.00025,.000125):
                    drive = Drive(cap)
                    rows = []
                    def observe(t,m,d,edge):
                        load = 0.
                        for i,c in enumerate(d.contact):
                            ids = list(c.geom)
                            table = m.geom("table").id
                            if table in ids:
                                w = np.zeros(6)
                                mj.mj_contactForce(m,d,i,w)
                                load += (1 if ids[0]==table else -1)*(c.frame.reshape(3,3).T@w[:3])[2]
                        row = (t,edge,d.qpos[2]*1000,drive.error*1000,load)
                        writer.writerow([j+1,cap,dt*1000,*row])
                        rows.append(row)
                    trace = replay(pose,0,dt,("body","wheels"),observe=observe,drive=drive)
                    a = np.array(rows)
                    print(j+1,cap,dt,"peak load, peak error, final displacement",a[:,4].max(),abs(a[:,3]).max(),trace[-1,1])
                    if dt==.000125:
                        for ax,k in zip(axes[:,j],(1,3,4)):
                            ax.plot(a[:,0],a[:,k],label=f"{cap:g} N limit")
    for j,title in enumerate(("Earlier cube sighting","Final cube sighting")):
        axes[0,j].set_title(title)
        axes[-1,j].set_xlabel("Seconds after command")
    for ax,label in zip(axes[:,0],("Center past edge (mm)","Chassis lag (mm)","Shelf force (N)")):
        ax.set_ylabel(label)
    axes[0,0].legend()
    fig.savefig(here/(output_name+".png"),dpi=150)
