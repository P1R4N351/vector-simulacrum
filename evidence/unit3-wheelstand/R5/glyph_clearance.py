"""Estimate the height of the cube center and its lowest envelope corner from the top glyph."""
import itertools
import sys

from pickup_onset import action_time
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
glyph = np.genfromtxt(here/"glyph-raise.csv", delimiter=",", names=True)
intrinsics = np.array([[340.,0,209.5],[0,340.,119.5],[0,0,1]])
square = np.array([[-.5,-.5,0],[.5,-.5,0],[.5,.5,0],[-.5,.5,0]])
# The ordered border points give a normal pointing into the cube.
box = np.array(list(itertools.product([-22.,22.], [-22.,22.], [0.,44.])))

def pose(points):
    ok, rot, trans = cv2.solvePnP(square,points,intrinsics,None,flags=cv2.SOLVEPNP_ITERATIVE)
    assert ok and trans[2,0] > 0
    return cv2.Rodrigues(rot)[0],trans.ravel()

def heights(reference, target, width):
    base, origin = reference
    rotation, translation = target
    up = -base[:,2]
    top = (translation-origin)*width
    center_rise = up @ (top+22*(rotation[:,2]-base[:,2]))
    clearance = 44 + np.min((top+box @ rotation.T) @ up)
    return center_rise,clearance

rng = np.random.default_rng(291)
rows = []
for i,g in enumerate(glyph):
    picks = np.array([[g[f"x{j}"],g[f"y{j}"]] for j in range(1,5)])
    estimate = pose(picks)
    draws = [pose(picks+rng.uniform(-1,1,(4,2))) for _ in range(500)]
    if i == 0:
        reference, reference_draws = estimate, draws
    nominal = heights(reference,estimate,23)
    noisy = [heights(a,b,23) for a,b in zip(reference_draws,draws)]
    small = heights(reference,estimate,20)
    rows.append([g["frame"],g["wrist_mm"],*nominal,*np.percentile(noisy,[5,95],axis=0).T.ravel(),*small])
rows = np.array(rows)
lights = np.genfromtxt(here/"raise-lights.csv",delimiter=",",names=True)
start = action_time(here/"actions.csv","pickup for inward transport","sec")
trace = np.genfromtxt(Path(sys.argv[1])/"pickup-replay.csv",delimiter=",",names=True)
times = np.interp(rows[:,0],lights["frame"],lights["receipt_s"])-start
height = np.interp(times,trace["sec"],trace["center_height_mm"])
# Reference the shelf-supported pre-command center; it may already have tipped by frame 85.
baseline = np.median(trace["center_height_mm"][(trace["sec"]>=-.25)&(trace["sec"]<0)])
rows = np.column_stack([rows,height-baseline])
np.savetxt(here/"glyph-clearance.csv",rows,delimiter=",",comments="",fmt="%.4f",
           header="frame,wrist_mm,center_rise_mm,envelope_clearance_mm,center_p05,center_p95,clearance_p05,clearance_p95,center_rise_20mm,clearance_20mm,sim_center_rise_mm")
fig,ax = plt.subplots(figsize=(7,4),layout="constrained")
for col,label in [(2,"Center rise"),(3,"Lowest corner above initial shelf plane")]:
    ax.plot(rows[:,1],rows[:,col],"o",label=label)
    ax.vlines(rows[:,1],rows[:,4+2*(col-2)],rows[:,5+2*(col-2)],alpha=.6)
ax.plot(rows[:,1],rows[:,-1],"x",color="black",label="Simulated center above settled baseline")
ax.axhline(0,color=".4",ls=":")
ax.set(xlabel="Recorded wrist height (mm)",ylabel="Estimated height change or clearance (mm)")
ax.legend(fontsize=8)
fig.savefig(here/"glyph-clearance.png",dpi=160)
