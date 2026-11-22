"""Midpoint test of fixed-attitude translation during the late roll signal."""
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import cumulative_trapezoid

here = Path(__file__).resolve().parent
camera = np.array([64.5, 64.75, 65.0])
# Same far-right rubber corner on three overhead frames, in cropped pixels.
picks = np.float32([[46, 8], [46, 8], [48, 9]])
# Existing 23 mm top glyph registration in half-resolution camera pixels.
glyph = np.float32([[170,187],[330,157],[361,242],[184,269]])/8 + [282,295]
world = np.float32([[0,0],[23,0],[23,23],[0,23]])
# The video patch at (10,8) matches the docked crop at (30,43).
offset = np.float32([260+20,265+35])


def optical(points, corners):
    transform = cv2.getPerspectiveTransform(np.float32(corners), world)
    p = cv2.perspectiveTransform(np.float32(points[None] + offset), transform)[0]
    return np.linalg.norm((p[0]+p[2])/2-p[1])


def packet_test(directory, clock):
    zero = next(float(r['sec']) for r in csv.DictReader((directory/'actions.csv').open())
                if r['action']=='roll cube')
    rows = np.genfromtxt(directory/'cube.csv.gz', names=True, delimiter=',')
    t = rows['sec'][::3]-zero
    y = rows['y'].reshape(-1,3).mean(axis=1)/8192
    tt = np.linspace(camera[0]-clock,camera[-1]-clock,501)
    acceleration = (np.interp(tt,t,y)-np.median(y[(t>=-1)&(t<0)]))*9806.65/.970709
    velocity = cumulative_trapezoid(acceleration,tt,initial=0)
    position = cumulative_trapezoid(velocity,tt,initial=0)
    return abs(position[-1]/2-position[250])


if __name__=='__main__':
    rng = np.random.default_rng(18)
    draw = [optical(picks+rng.uniform(-2,2,picks.shape), glyph+rng.uniform(-1,1,glyph.shape))
            for _ in range(4000)]
    fig, axes = plt.subplots(1,3,figsize=(8,3),layout='constrained')
    vid = cv2.VideoCapture(str(here/'cube-settling.mkv'))
    for ax,t,point in zip(axes,camera,picks):
        vid.set(cv2.CAP_PROP_POS_FRAMES,round((t-58)*12))
        ok,frame = vid.read()
        assert ok
        ax.imshow(frame[:,:,0],cmap='gray',vmin=0,vmax=255,interpolation='nearest')
        ax.plot(*point,'+',c='#ff794d',ms=12)
        ax.set_title(f'Camera {t:.2f} s')
        ax.axis('off')
    vid.release()
    fig.savefig(here/'slide-check.png',dpi=170)
    clocks=np.linspace(59.754,59.875,51)
    rows=[('Visible corner',optical(picks,glyph),*np.percentile(draw,[5,95]))]
    for directory in [here,here.parent/'R9']:
        values=[packet_test(directory,c) for c in clocks]
        rows.append((directory.name+' fixed attitude, sensor Y',np.median(values),min(values),max(values)))
    with (here/'slide-check.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['measurement','central_mm','low_mm','high_mm'])
        w.writerows(rows)
