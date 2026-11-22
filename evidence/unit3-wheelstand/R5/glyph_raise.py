"""Estimate top-face normal change from the square printed glyph border."""
import csv
from pathlib import Path

import cv2
import numpy as np

here = Path(__file__).resolve().parent
sheet = cv2.imread(str(here/"raise-side.jpg"))
with (here/"raise-lights.csv").open() as stream:
    times = list(csv.DictReader(stream))
# Manually picked border centerline corners, clockwise from far left.
# Coordinates are in 3x enlarged 70x50 crops centered at the listed pixels.
picks = [
    (0,108,155,[(74,32),(174,36),(151,109),(42,104)]),
    (1,107,155,[(66,33),(169,36),(146,109),(32,104)]),
    (2,107,155,[(66,33),(167,36),(144,109),(32,104)]),
    (3,109,152,[(55,24),(160,26),(147,116),(32,112)]),
    (6,174,90,[(39,14),(161,15),(155,124),(28,124)]),
    (7,200,86,[(35,19),(159,19),(157,128),(25,126)]),
]
k = np.array([[340.,0,209.5],[0,340.,119.5],[0,0,1]])
square = np.array([[-.5,-.5,0],[.5,-.5,0],[.5,.5,0],[-.5,.5,0]])

def normal(corners):
    ok, rot, trans = cv2.solvePnP(square,corners,k,None,flags=cv2.SOLVEPNP_ITERATIVE)
    assert ok and trans[2,0]>0
    projected = cv2.projectPoints(square,rot,trans,k,None)[0].reshape(4,2)
    return cv2.Rodrigues(rot)[0][:,2], np.sqrt(np.mean(np.sum((projected-corners)**2,axis=1)))

rng = np.random.default_rng(75)
rows, views = [], []
for i,cx,cy,local in picks:
    corners = (np.array(local,dtype=float)+.5)/3 - .5 + [cx-35,cy-25]
    axis, error = normal(corners)
    draws = np.array([normal(corners+rng.uniform(-1,1,(4,2)))[0] for _ in range(500)])
    if i==0:
        reference, reference_draws = axis, draws
    change = np.degrees(np.arccos(np.clip(reference@axis,-1,1)))
    varied = np.degrees(np.arccos(np.clip(np.sum(reference_draws*draws,axis=1),-1,1)))
    row = times[i]
    rows.append([row["frame"],row["wrist_mm"],*corners.ravel(),change,*np.percentile(varied,[5,95]),error])
    x,y = (i%2)*420,(i//2)*264
    tile = sheet[y:y+264,x:x+420].copy()
    cv2.polylines(tile,[np.rint(corners).astype("int32")],True,(0,220,255),1,cv2.LINE_AA)
    views.append(tile)
cv2.imwrite(str(here/"glyph-raise.jpg"),np.vstack([np.hstack(views[j:j+2]) for j in range(0,6,2)]))
with (here/"glyph-raise.csv").open("w") as stream:
    writer=csv.writer(stream)
    writer.writerow("frame wrist_mm x1 y1 x2 y2 x3 y3 x4 y4 normal_change_deg jitter_p05_deg jitter_p95_deg fit_rms_px".split())
    writer.writerows(rows)
