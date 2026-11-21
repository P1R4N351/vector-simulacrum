"""Transfer one upper-left outer glyph corner between two pre-contact head poses."""
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

here = Path(__file__).resolve().parent
folder = here / 'E/precontact'
rows = json.loads((folder/'inputs.json').read_text())
cal = json.loads((here.parents[1]/'web/camera_calibration_vector2.json').read_text())
f = np.array([cal['fx_px'], cal['fy_px']])
principal = np.array([cal['cx_px'], cal['cy_px']])
sdk = np.array([136.9086, -24.097736])
angle = np.deg2rad(88.9)
normal = np.array([np.cos(angle), np.sin(angle)])
right = np.array([-normal[1], normal[0]])
corner_offset = 22*normal - 11.5*right

def camera(row):
    h, yaw = row['head'], row['yaw']
    c, s = np.cos(h), np.sin(h)
    forward = np.array([np.cos(yaw), np.sin(yaw), 0])
    left = np.array([-np.sin(yaw), np.cos(yaw), 0])
    up = np.array([0, 0, 1])
    p = h - np.deg2rad(4)
    origin = np.array([row['x'], row['y'], 49+18.84*s-7.96*c]) + forward*(-13+18.84*c+7.96*s)
    basis = np.stack((-left, forward*np.sin(p)-up*np.cos(p), forward*np.cos(p)+up*np.sin(p)))
    return origin, basis

def register(row, pixel, height=33.5):
    origin, basis = camera(row)
    ray = np.r_[(pixel-principal)/f, 1] @ basis
    return (origin+ray*((height-origin[2])/ray[2]))[:2]-corner_offset

def project(row, center):
    origin, basis = camera(row)
    ray = basis @ (np.r_[center+corner_offset, 33.5]-origin)
    return principal+f*ray[:2]/ray[2]

fit = register(rows[0], rows[0]['corner'])
fig, axes = plt.subplots(1,2, figsize=(9,4), layout='constrained')
for i,(ax,row) in enumerate(zip(axes, rows)):
    actual = np.array(row['corner'])
    ax.imshow(Image.open(folder/row['file']))
    for pt,mark,color,label in [(actual,'+','white','Observed corner'),(project(row,sdk),'x','#ff9955','Earlier SDK pose'),(project(row,fit),'o','#47e6fa','Head-down estimate')]:
        ax.plot(*pt,mark,color=color,ms=11,mew=2,mfc='none',label=label)
    ax.set(xlim=(485,685),ylim=(465,170),title=['Head down · registration','Head raised · held out'][i])
    print(row['timestamp'], 'pixel residual', (project(row,fit)-actual).round(1), 'position relative to SDK', (register(row,actual)-sdk).round(2))
axes[1].legend(fontsize=8)
fig.savefig(folder/'overlay.png',dpi=155)
for dz in (-1,1):
    print('height sensitivity mm',dz,(register(rows[0],rows[0]['corner'],33.5+dz)-fit).round(2))

# Intersect plan-view bearings. This does not assume a common camera elevation.
early = dict(x=26.210676,y=54.10677,yaw=-0.039135545,head=-0.22526754,corner=[577,255])
def planar_bearing(row):
    origin,basis = camera(row)
    ray = np.r_[(np.array(row['corner'])-principal)/f, 1] @ basis
    return origin[:2], ray[:2]
o1,b1 = planar_bearing(early)
o2,b2 = planar_bearing(rows[0])
ranges = np.linalg.solve(np.column_stack([b1,-b2]),o2-o1)
cross = o1+ranges[0]*b1-corner_offset
print('two-view offset and held-out pixel residual', (cross-sdk).round(2), (project(rows[1],cross)-rows[1]['corner']).round(1),'ray depths',ranges)
for row in [early,*rows]:
    origin,basis = camera(row)
    ray = np.r_[(np.array(row['corner'])-principal)/f,1] @ basis
    delta = cross+corner_offset-origin[:2]
    height = origin[2]+ray[2]*(delta @ ray[:2])/(ray[:2] @ ray[:2])
    print('bearing intersection implied height', round(height,2))
