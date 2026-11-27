"""Closest fork surface to the exposed underside of the acquisition face's upper rim."""
import csv
import importlib
import sys
from xml.etree import ElementTree as ET

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

run = importlib.import_module("fork-rake")
tree = ET.parse(sys.argv[1])
run.require_printed_fork(tree)
cube = tree.find(".//body[@name='cube']")
boxes = []
for geom in cube.findall("geom"):
    assert geom.get("type") == "box" and geom.get("quat") is None
    p,s = [1000*np.fromstring(geom.get(k),sep=" ") for k in ("pos","size")]
    boxes.append((p-s,p+s))
# The center opening ends at the lower face of the top central shell tile.
z = min(lo[2] for lo,hi in boxes if lo[2]>0 and lo[0]<0<hi[0] and lo[1]<0<hi[1])
# Tile the forwardmost 8 mm of that ledge, covering both corner openings.
x0 = min(lo[0] for lo,hi in boxes if lo[2]<z+.01<hi[2])
xs = sorted({x0,x0+8,*[v for b in boxes for pt in b for v in [round(pt[0],8)] if x0<v<x0+8]})
ys = sorted({round(pt[1],8) for b in boxes for pt in b})
def filled(x,y,h):
    return any(np.all(lo<[x,y,h]) and np.all(np.array([x,y,h])<hi) for lo,hi in boxes)
rects=[]
for a,b in zip(xs,xs[1:]):
    if b-a<1e-6: continue
    for c,e in zip(ys,ys[1:]):
        u,v=(a+b)/2,(c+e)/2
        if e-c<1e-6 or not filled(u,v,z+.01) or filled(u,v,z-.01): continue
        if rects and rects[-1][:2]==[a,b] and abs(rects[-1][3]-c)<1e-7:
            rects[-1][3]=e
        else:
            rects.append([a,b,c,e])
# A conservative interior strip common to the full sampled depth on each side.
regions=[("full",r) for r in rects]
for sign in (1,-1):
    side_rects=[r for r in rects if sign*(r[2]+r[3])>0]
    regions.append(("interior",[x0+1,x0+7,max(r[2] for r in side_rects)+1,min(r[3] for r in side_rects)-1]))
probes=[]
for j,(region,(a,b,c,e)) in enumerate(regions):
    name=f"under_rim_{j}"
    probes.append((name,("+Y" if c>0 else "-Y",region)))
    # 2 µm thick and zero mass. Contact masks exclude these from dynamics.
    ET.SubElement(cube,"geom",name=name,type="box",mass="0",contype="0",conaffinity="0",
                  pos=f"{(a+b)/2000} {(c+e)/2000} {z/1000}",
                  size=f"{(b-a)/2000} {(e-c)/2000} .000001")
groups=sorted({group for _,group in probes})
run.add_links(tree,run.repo)
m=mj.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding="unicode"))
fork=[i for i in range(m.ngeom) if m.geom_bodyid[i]==m.body("fork").id and m.geom_type[i]==mj.mjtGeom.mjGEOM_MESH]
rows=[]
def observe(m,d,t,i):
    if not (2.70<=t<=3.45) or i%round(.005/m.opt.timestep): return
    for group in groups:
        best=float("inf")
        connector=np.zeros(6)
        for name,s in probes:
            if s!=group: continue
            for geom in fork:
                dist=mj.mj_geomDistance(m,d,geom,m.geom(name).id,.1,connector)
                if dist<best:
                    best=dist
                    local=d.xmat[m.body("cube").id].reshape(3,3).T@(connector[:3]-connector[3:])
                    offset=local.copy()*1000
                    rim=1000*(d.xmat[m.body("cube").id].reshape(3,3).T@(connector[3:]-d.xpos[m.body("cube").id]))
        rows.append([m.opt.timestep*1000,round(t,5),group[0],best*1000,*offset,group[1],*rim])
summaries=list(run.replay(m,[0],observe=observe))
for new,baseline in zip(summaries,np.genfromtxt(run.here/"fork-rake.csv",delimiter=",",names=True)[::5]):
    np.testing.assert_allclose(new,list(baseline),atol=1e-6)
with (run.here/"pickup-undercut.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["step_ms","time_s","cube_side","clearance_mm","fork_minus_rim_x_mm","fork_minus_rim_y_mm","fork_minus_rim_z_mm","region","rim_x_mm","rim_y_mm","rim_z_mm"])
    w.writerows(rows)
fig,(ax,xy)=plt.subplots(1,2,figsize=(11,4.3),layout="constrained")
for dt in [1,.5]:
    for s,col in [("+Y","tab:blue"),("-Y","tab:orange")]:
        r=[a for a in rows if a[0]==dt and a[2]==s and a[7]=="full"]
        ax.plot([a[1] for a in r],[a[3] for a in r],"-" if dt==1 else "--",color=col,label=f"{s}, {dt:g} ms")
r=[a for a in rows if a[0]==1 and a[2]=="+Y" and a[7]=="interior"]
ax.plot([a[1] for a in r],[a[3] for a in r],color="tab:green",label="+Y interior strip, 1 ms")
ax.axvspan(3.11,3.38,color="gray",alpha=.18,label="Visible tipping bracket")
ax.axvline(3.164,color="k",ls=":",label="Last loaded fork contact (1 ms replay)")
ax.set(xlabel="Seconds since pickup command",ylabel="Nearest fork surface to rim underside (mm)")
ax.legend(ncol=2,fontsize=8)
from matplotlib.patches import Rectangle
for region,(a,b,c,e) in regions:
    if c<0: continue
    xy.add_patch(Rectangle((a,c),b-a,e-c,facecolor=("lightgray" if region=="full" else "silver"),edgecolor="none"))
for t in (3.01,3.11,3.165,3.38):
    point=next(a for a in rows if a[0]==1 and a[1]==t and a[2]=="+Y" and a[7]=="full")
    x,y=point[8:10]
    u,v=point[4:6]
    xy.plot([x,x+u],[y,y+v],"-",color="gray",lw=1)
    xy.scatter(x+u,y+v,label=f"+{t:g} s",s=22)
xy.set(aspect="equal",xlabel="Cube X (mm)",ylabel="Cube Y (mm)",title="Nearest fork point, top view")
xy.autoscale()
xy.legend(fontsize=8)
fig.savefig(run.here/"pickup-undercut.png",dpi=150)
print("rectangles",rects,"height",z)
