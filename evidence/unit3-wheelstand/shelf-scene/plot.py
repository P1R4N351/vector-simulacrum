"""Plot the same shelf outline, dock surfaces and native map as the web scene."""
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection
from matplotlib.lines import Line2D
from scipy.spatial.transform import Rotation

from simulacrum import dock, mjcf
from simulacrum.camera_spins import project

here = Path(__file__).resolve().parent
survey = mjcf.shelf_survey()
data = json.loads((here.parents[2]/"web/public/robot/vector2/shelf-navmap.json").read_text())
xy = np.array(survey["outline_mm"])
p = np.array(survey["camera_pose"])
fig, (photo, plan) = plt.subplots(1, 2, figsize=(12, 5.7), layout="constrained")
photo.imshow(plt.imread(here/"survey.jpg"))
for i, a in enumerate(xy):
    b = xy[(i+1) % len(xy)]
    line = np.linspace(a,b,50)
    pix = project(np.c_[line[:,1],-line[:,0]-160,np.zeros(50)], p)/2
    estimated = i in (2,3,4,5)
    color = "#ee9933" if estimated else "#32bdb2"
    photo.plot(*pix.T, color=color, linestyle="--" if estimated else "-", lw=2)
    plan.plot(*np.stack((a,b)).T,color=color,linestyle="--" if estimated else "-",lw=2)
photo.set(xlim=(75,815),ylim=(540,210),title="Shelf outline against the upright See3CAM frame")
photo.axis("off")
lip, _, _ = survey["dock_pose_mm"]
origin = lip-dock.DOCKED_START_DIST_MM
for box in dock.load_surface_boxes():
    hx,hy,_ = box.half_mm
    corners = [[x,y,0] for x,y in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]]
    w,x,y,z = box.quat
    pts = Rotation.from_quat([x,y,z,w]).apply(corners) + box.center_mm
    plan.fill(lip-pts[:,0],-pts[:,1],color="#444754",alpha=.4)
colors = {2:"#0f9c86",7:"#d33e41",3:"#ac6ace",4:"#5280b4",5:"#5280b4",6:"#777777"}
for kind in colors:
    cells = []
    if kind in (2,7):
        for i,j,k,_ in data["cells"]:
            if k==kind:
                cells.append([(origin+10*(i+a),10*(j+b)) for a,b in ((0,0),(1,0),(1,1),(0,1))])
    else:
        for x,y,w,k,q in data["latest_objects_and_visibility"]:
            if k!=kind: continue
            c,s=np.cos(q),np.sin(q)
            cells.append([(origin+x+w*(a*c-b*s),y+w*(a*s+b*c)) for a,b in ((-.5,-.5),(.5,-.5),(.5,.5),(-.5,.5))])
    plan.add_collection(PolyCollection(cells,facecolors=colors[kind],edgecolors="none",alpha=.65))
plan.set(aspect="equal",xlim=(-240,225),ylim=(-260,250),xlabel="Outward from dock (mm, scene frame)",ylabel="Left (mm)",title="Unit 3 native map registered to the dock")
handles=[Line2D([],[],color=c,lw=6,label=t) for c,t in [("#0f9c86","Clear of cliff"),("#ac6ace","Mapped object"),("#5280b4","Proximity"),("#444754","Dock")]]
plan.legend(handles=handles,fontsize=8,loc="upper right")
photo.set_title("See3CAM   •   dashed edge is inferred")
fig.savefig(here/"shelf-map.png",dpi=160)
