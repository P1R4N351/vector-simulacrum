"""Reproduce from the two stationary Record3D captures, e.g. python check.py CAPTURE_ROOT."""
import json
import sys
from pathlib import Path

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

out = Path(__file__).resolve().parent
fig, axes = plt.subplots(2, 2, figsize=(10, 7), constrained_layout=True)
results = {}
for i, name in enumerate(("side", "pass2/offset")):
    source = Path(sys.argv[1]) / name
    meta = json.loads((source / "metadata.json").read_text())
    width, height = meta["originalSize"]
    k = meta["K"]
    f = np.array([k[4]*640/height, k[0]*480/width])
    principal = np.array([k[7]*640/height, (width-1-k[6])*480/width])
    def landscape(a, interpolation):
        return cv.resize(cv.rotate(a, cv.ROTATE_90_COUNTERCLOCKWISE), (640, 480), interpolation=interpolation)
    frames = sorted(source.glob("*-color.jpg"))
    im = np.uint8(np.median([landscape(cv.imread(str(p)), cv.INTER_LINEAR) for p in frames], axis=0))
    depth = np.stack([landscape(np.load(p), cv.INTER_NEAREST)*1000 for p in sorted(source.glob("*-depth.npy"))])
    mask = cv.inRange(cv.cvtColor(im, cv.COLOR_BGR2HSV), (115,60,60), (159,255,255))
    fits = []
    for contour in cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)[0]:
        if cv.contourArea(contour) < 200:
            continue
        edge = contour.reshape(-1,2)
        fits.append(cv.fitEllipse(edge[(edge[:,0]>1)&(edge[:,0]<638)]))
    fits.sort(key=lambda e:max(e[1]))
    ax, dz = axes[i]
    ax.imshow(cv.cvtColor(im, cv.COLOR_BGR2RGB))
    depth = np.ma.masked_less_equal(depth, 0)
    dz.imshow(np.ma.median(depth, axis=0), cmap="viridis", vmin=60, vmax=350)
    def sample(uv):
        x,y = np.rint(uv).astype(int)
        patch = depth[:,y-4:y+5,x-4:x+5]
        z = np.ma.median(patch, axis=(1,2)).filled(np.nan)
        ray = np.r_[(uv-principal)/f, 1]
        return z[:,None]*ray, np.count_nonzero(np.isfinite(z))
    points, scaled, counts = [], [], []
    for ellipse, diameter in zip(fits, (18.5,26), strict=True):
        uv = np.array(ellipse[0])
        xyz, count = sample(uv)
        points.append(xyz)
        counts.append(int(count))
        scaled.append(np.r_[(uv-principal)/f,1]*np.mean(f)*diameter/max(ellipse[1]))
        for a in (ax,dz):
            a.plot(*uv, marker="+", color="white", ms=11)
    def span(p):
        return np.linalg.norm(p[0]-p[1],axis=-1)
    valid = span(points)
    res = dict(frames=len(frames), rear_front_valid_frames=counts, ellipse_fits=fits,
               lidar_separation_p10_median_p90_mm=np.percentile(valid[np.isfinite(valid)],[10,50,90]).tolist() if np.isfinite(valid).any() else None,
               diameter_scaled_rgb_separation_mm=float(span(scaled)))
    if i:
        # Top border is visible. Depth sampled just inside each end to avoid background mixing.
        ends = np.array([[329,101],[401,101]])
        measured = []
        for uv, interior in zip(ends, [[336,107],[394,107]]):
            xyz,_ = sample(np.array(interior))
            measured.append(xyz[:,2,None]*np.r_[(uv-principal)/f,1])
        res["glyph_top_edge_p10_median_p90_mm"] = np.nanpercentile(span(measured),[10,50,90]).tolist()
        for a in (ax,dz):
            a.plot(*ends.T, color="orange", lw=2)
    results[name]=res
    for a in (ax,dz):
        a.set_xlim((315,639) if i==0 else (0,415))
        a.set_ylim(430,75)
        a.axis("off")
    ax.set_title(["Right-facing • color","Left-facing • color"][i])
    dz.set_title("Registered depth • black gaps have no return")
    dz.set_facecolor("black")
    dz.images[0].set_cmap(plt.get_cmap("viridis").with_extremes(bad="black"))
fig.savefig(out/"comparison.jpg", dpi=150)
(out/"measurements.json").write_text(json.dumps(results,indent=2)+"\n")
