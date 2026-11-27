"""Compare shifts of visible far rubber and glyph landmarks before occlusion."""
import csv
from pathlib import Path
import cv2
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
frames=[78,84,85,86,87]
# Crop pixel coordinates: far-right rubber crest, upper-left and upper-right glyph bends.
points=np.array([[[49,5],[23,15],[43,12]],[[52,7],[25,19],[47,15]],
                 [[54,9],[26,22],[48,18]],[[55,11],[26,25],[49,21]],
                 [[56,15],[29,28],[50,24]]],float)
cap=cv2.VideoCapture(str(here/'cube-settling.mkv'))
assert cap.get(cv2.CAP_PROP_FPS)==12
fig,axes=plt.subplots(1,len(frames),figsize=(12,3),layout='constrained')
colors=['#e07a35','#2fbbcc','#97c45d']
for ax,i,p in zip(axes,frames,points):
    cap.set(cv2.CAP_PROP_POS_FRAMES,i)
    ok,bgr=cap.read()
    assert ok
    ax.imshow(bgr[:,:,0],cmap='gray',vmin=0,vmax=255,interpolation='nearest')
    for xy,c in zip(p,colors):
        ax.scatter(*xy,marker='+',s=70,c=c)
    ax.set(xlim=(10,62),ylim=(38,0),title=f'{58+i/12:.3f} s')
    ax.set_axis_off()
cap.release()
fig.savefig(here/'terminal-landmarks.png',dpi=180)
def rigid_shift_score(displacement):
    mean=displacement.mean(axis=-2,keepdims=True)
    return np.sqrt(np.mean(np.sum((displacement-mean)**2,axis=-1),axis=-1))
def difference(p):
    return (p[...,1,:]+p[...,2,:])/2-p[...,0,:]
disp=points[-1]-points[0]
err=rigid_shift_score(disp)
delta=difference(disp)
print('Observed mean shift, translation residual RMS, relative glyph shift',disp.mean(axis=0),err,delta)
rng=np.random.default_rng(812)
with (here/'terminal-landmarks.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['camera_s','feature','x_px','y_px','dx_px','dy_px'])
    for frame,p in zip(frames,points):
        for i,label in enumerate(('far_right_crest','left_glyph_bend','right_glyph_bend')):
            w.writerow([58+frame/12,label,*p[i],*(p[i]-points[0,i])])
# What apparent residual could arise from a common image translation with pick error alone?
for box in [1,1.5,2]:
    jitter=rng.uniform(-box,box,(20000,2,3,2))
    noise=jitter[:,1]-jitter[:,0]
    print(box,'95th percentile pure translation residual',np.percentile(rigid_shift_score(noise),95),
          'glyph vertical relative-shift sensitivity',np.percentile(delta[1]+difference(noise)[:,1],[5,95]))
