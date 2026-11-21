"""Replay the original See3CAM pose while varying only the shelf perpendicularity weight."""
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import brentq

root = next((p for p in Path(__file__).resolve().parents if (p / 'see3cam_shelf.py').exists()), Path.cwd())
sys.path.insert(0, str(root))
import see3cam_shelf as shelf

# B image centers in the survey's rectified See3CAM crop (crop origin in full frame).
HUB = np.array([[[470,509], [628,552]], [[705,555], [532,550]]],float)
ORIGIN = np.array([310.,395.])
Z = np.array([14.5,10.75])

def hub_positions(uv, pose, lens, delta=0):
    return np.array([shelf.locate(pixel,height+delta,pose,lens)[0] for pixel,height in zip(uv,Z)])

def camera_origin(p):
    return Rotation.from_rotvec(-p[:3]).apply(-p[3:6])

def spans(p,lens,hub):
    return [np.linalg.norm(np.diff(hub_positions(uv,p,lens),axis=0)) for uv in hub]

def metrics(p,lens):
    errors = shelf.image_points(shelf.WORLD,p,lens)-shelf.GLYPH
    a,b,c,d = shelf.locate(shelf.SHELF,0,p,lens)
    across = (b-a)/np.linalg.norm(b-a)
    components = np.array([(c-a)@across,(d-b)@across])
    return np.sqrt(np.mean(np.sum(errors**2,axis=1))), components

def main():
    lens = shelf.lens_fit(shelf.edge_groups())
    hub = shelf.distort(HUB+ORIGIN,lens)
    for w in [1,5,20,50,100,200,1e6]:
        p=shelf.camera_pose(lens,edgeweight=w)
        residual,components=metrics(p,lens)
        print(w, np.round([*spans(p,lens,hub),residual,camera_origin(p)[2],*components],3))
    original = shelf.camera_pose(lens)
    for v in hub:
        needed=brentq(lambda h:np.linalg.norm(np.diff(hub_positions(v,original,lens,h),axis=0))-48,-30,0)
        print('required shift in both center heights (mm)',round(needed,2))

if __name__=='__main__':
    main()
