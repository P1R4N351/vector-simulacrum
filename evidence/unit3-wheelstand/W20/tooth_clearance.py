"""Clip a lateral fork upright at the crossbar and locate actual cube contacts."""
import csv
from vertical_fork import ForkDrive
from free_departure import here, mj, np, plt, poses, replay

cut = -.0176
class DamageTrial(ForkDrive):
    def __init__(self, side):
        super().__init__(1., False)
        self.side = side

    def build(self, root, forward):
        super().build(root, forward)
        fork = root.find(".//body[@name='fork']")
        for geom in list(fork.findall("geom[@mesh]")):
            mesh = root.find(f".//asset/mesh[@name='{geom.get('mesh')}']")
            v = np.fromstring(mesh.get("vertex"), sep=" ").reshape(-1,3)
            if not self.side or min(v[:,1]*self.side)<=0 or max(v[:,2])<=cut:
                continue
            if min(v[:,2])>=cut:
                fork.remove(geom)
                continue
            faces = np.fromstring(mesh.get("face"),sep=" ",dtype=int).reshape(-1,3)
            points = list(v[v[:,2]<=cut])
            for triangle in faces:
                for i,j in zip(triangle,np.roll(triangle,1)):
                    a,b = v[i],v[j]
                    if (a[2]-cut)*(b[2]-cut)<0:
                        points.append(a+(b-a)*((cut-a[2])/(b[2]-a[2])))
            points = np.unique(points,axis=0)
            mesh.set("vertex"," ".join(map(str,points.ravel())))
            # MuJoCo builds the convex hull from the retained vertices.
            mesh.attrib.pop("face")

fig, axes = plt.subplots(1,2,figsize=(10,4),sharey=True,layout="constrained")
with (here/"tooth-clearance.csv").open("w") as f:
    writer = csv.writer(f,lineterminator="\n")
    writer.writerow("pose variant step_ms sec encoder_derived_mm wrist_mm cube_height_mm contact_hulls".split())
    for j,pose in enumerate(poses[poses["run"]=="W20"][-2:]):
        for side,label in ((0,"intact"),(1,"left clipped"),(-1,"right clipped")):
            for dt in (.00025,.000125):
                drive = DamageTrial(side)
                rows = []
                def observe(t,m,d,edge):
                    hits = set()
                    for c in d.contact:
                        names = [m.geom(g).name for g in c.geom]
                        if any(name.startswith("cube_") for name in names):
                            hits.update(name.rsplit("_",1)[1] for name in names if name.startswith("lift_fork_geom_stl_"))
                    row = [t,1000*drive.last_z,1000*d.joint("vertical").qpos[0],1000*d.qpos[2]]
                    writer.writerow([j+1,label,dt*1000,*row," ".join(sorted(hits,key=int))])
                    rows.append(row)
                replay(pose,0,dt,("body","wheels"),observe=observe,drive=drive)
                a = np.array(rows)
                if dt==.000125:
                    axes[j].plot(a[:,0],a[:,2],label=label)
                    if not side:
                        axes[j].plot(a[:,0],a[:,1],"k--",label="Encoder-derived")
        axes[j].set(title=("Earlier" if j==0 else "Final")+" cube sighting",xlabel="Seconds after request")
axes[0].set_ylabel("Wrist height (mm)")
axes[1].legend()
fig.savefig(here/"tooth-clearance.png",dpi=150)

# Show both clipping variants and the first-contact hulls in the fork frame.
from copy import deepcopy
from free_departure import tree, add_front
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull

fig, axes = plt.subplots(1,3,figsize=(10,3.6),sharex=True,sharey=True,layout="constrained")
with (here/"tooth-clearance.csv").open() as stream:
    records = list(csv.DictReader(stream))
initial = set()
for pose_id in ("1","2"):
    hit = next(r for r in records if r["pose"]==pose_id and r["variant"]=="intact" and r["contact_hulls"])
    initial.update(hit["contact_hulls"].split())
for ax,side,title in zip(axes,(0,1,-1),("Intact","Left upright clipped","Right upright clipped")):
    root = deepcopy(tree.getroot())
    add_front(root,("body","wheels"))
    DamageTrial(side).build(root,np.array([1.,0.]))
    for geom in root.findall(".//body[@name='fork']/geom[@mesh]"):
        mesh = root.find(f".//asset/mesh[@name='{geom.get('mesh')}']")
        points = np.fromstring(mesh.get("vertex"),sep=" ").reshape(-1,3)[:,1:]*1000
        hull = ConvexHull(points)
        hit = geom.get("name").rsplit("_",1)[1] in initial
        ax.add_patch(Polygon(points[hull.vertices],facecolor="#cd4821" if hit else "#9badbd",edgecolor="white",linewidth=.25,zorder=2 if hit else 1))
    ax.set(xlim=(26,-26),ylim=(-31,5),aspect="equal",title=title,xlabel="Lateral position (mm)")
axes[0].set_ylabel("Height relative to wrist (mm)")
fig.suptitle("Orange pieces touched the cube at the first sampled contact")
fig.savefig(here/"tooth-contact-map.png",dpi=160)
