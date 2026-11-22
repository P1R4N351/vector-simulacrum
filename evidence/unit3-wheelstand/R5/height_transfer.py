"""Compare frame-relative center rise; bound sensitivity to the unknown shelf normal."""
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
read = lambda name: np.genfromtxt(here / name, names=True, delimiter=",", dtype=None, encoding="utf8")
glyph, runs = read("glyph-raise.csv"), read("approach-links.csv")
K = np.array([[340.,0,209.5],[0,340.,119.5],[0,0,1]])
square = np.array([[-.5,-.5,0],[.5,-.5,0],[.5,.5,0],[-.5,.5,0]])

def solve(corners):
    ok, r, t = cv2.solvePnP(square, corners, K, None)
    assert ok and t[2,0] > 0
    return t.ravel(), cv2.Rodrigues(r)[0][:,2]

rng = np.random.default_rng(75)
rows = []
for row in glyph:
    corners = np.array([[row[f"x{i}"], row[f"y{i}"]] for i in range(1,5)])
    draws = [solve(corners)] + [solve(corners+rng.uniform(-1,1,(4,2))) for _ in range(500)]
    trans, inward = map(np.asarray, zip(*draws))
    if row["frame"] == 85:
        ref_t, ref_n = trans, inward
    for width in (20,23):
        shift = width*(trans-ref_t) + 22*(inward-ref_n)
        up = -ref_n
        rise = (shift*up).sum(axis=1)
        lateral = np.linalg.norm(shift-rise[:,None]*up, axis=1)
        # Minimum misalignment of the assumed shelf normal that cancels positive rise.
        angle = np.rad2deg(np.arctan2(rise,lateral))
        rows.append([int(row["frame"]), width, rise[0], *np.percentile(rise[1:],[5,95]),
                     angle[0], np.percentile(angle[1:],5)])
a = np.array(rows)
fig, ax = plt.subplots(figsize=(8,4.5), layout="constrained")
for width, offset in [(20,-.4),(23,.4)]:
    v = a[a[:,1]==width]
    h = glyph["wrist_mm"]+offset
    ax.plot(h,v[:,2],"o",ms=4,label=f"Filmed · {width} mm glyph")
    ax.vlines(h,v[:,3],v[:,4],alpha=.65)
for mode in np.unique(runs["link_condition"]):
    b = runs[(runs["link_condition"]==mode)&(runs["step_ms"]==.125)]
    height = b["sim_center_height_mm"]
    ax.plot(glyph["wrist_mm"],height-height[0],".-",label=mode)
ax.axhline(0,color=".5",ls=":")
ax.set(xlabel="Recorded wrist height (mm)",ylabel="Cube center rise from frame 85 (mm)")
ax.legend(fontsize=8,ncol=2)
fig.savefig(here/"height-transfer.png",dpi=160)
with (here/"height-transfer.csv").open("w") as stream:
    w = csv.writer(stream,lineterminator="\n")
    w.writerow("frame glyph_width_mm rise_mm rise_p05 rise_p95 zero_rise_axis_offset_deg offset_p05_deg".split())
    w.writerows(rows)
