"""Estimate front rail width in the glyph plane of the published cube photograph."""
from pathlib import Path
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt

# The published full-frame graphic has four orange landmarks with original
# image and metric coordinates in cube-face-photo.csv. Use them to transfer
# the original glyph registration to its graphic without a new scale fit.
base = Path(__file__).resolve().parent.parent / "unit3-printed-fork-contact"
image = cv2.imread(str(base / "cube-face-photo.png"))
rows = list(csv.DictReader((base / "cube-face-photo.csv").open()))
mask = cv2.inRange(image, (0,140,240), (20,180,255))
count, _, stats, centers = cv2.connectedComponentsWithStats(mask)
dots = sorted([c for c,s in zip(centers[1:], stats[1:]) if 40 < s[-1] < 150 and s[2] < 23 and c[1]>100], key=lambda p:p[0])

assert len(dots)==4
source=np.float32([dots[0],dots[1],dots[3],dots[2]])
target=np.float32([[float(r['face_x_mm']),float(r['face_z_mm'])] for r in rows])
H=cv2.getPerspectiveTransform(source,target)
def project(points):
    return cv2.perspectiveTransform(np.float32(points).reshape(-1,1,2),H)[:,0]
# Straight rail ends and outer pad boundaries picked in the published image.
# A two-pixel perturbation tests local picking sensitivity only.
picks = {
    "Upper rail": [(149,298),(666,353)],
    "Lower rail": [(169,526),(663,611)],
    "Upper rubber pads": [(138,243),(674,292)],
    "Lower rubber pads": [(166,577),(668,665)],
}
fig, ax = plt.subplots(figsize=(9, 6.5), layout="constrained")
ax.imshow(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
ax.set_axis_off()
offsets = np.array([[x,y] for x in (-2,2) for y in (-2,2)])
with open("photo-spans.csv","w",newline="") as stream:
    writer = csv.writer(stream,lineterminator="\n")
    writer.writerow(["feature","width_in_marker_plane_mm","pick_min_mm","pick_max_mm"])
    for label,pair in picks.items():
        left,right=np.asarray(pair)
        width=float(np.diff(project(pair)[:,0])[0])
        l=project(left+offsets)[:,0]
        r=project(right+offsets)[:,0]
        writer.writerow([label,round(width,3),round(float(min(r)-max(l)),3),round(float(max(r)-min(l)),3)])
        x,y=np.asarray(pair).T
        color = "#ff5da2" if "rubber" in label else "#1bdd8c"
        ax.plot(x,y,"o-",color=color,ms=4,lw=1.4)
        ax.text(np.mean(x),np.mean(y)-5,f"{label}  {width:.1f} mm",ha="center",color="white",
                fontsize=9,bbox=dict(facecolor="black",alpha=.75,pad=2))
ax.set_title("Apparent front spans registered on the 23 mm face marker")
fig.savefig("photo-spans.png",dpi=160)
