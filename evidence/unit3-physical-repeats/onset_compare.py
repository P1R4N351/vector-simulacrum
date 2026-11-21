"""Compare motion at independent camera receipts and display a frame sequence around onset."""
import csv
from pathlib import Path
import zipfile
import cv2
import matplotlib.pyplot as plt
import numpy as np
here=Path(__file__).resolve().parent/'E'
cam=np.genfromtxt(here/'onset-camera.csv',delimiter=',',names=True)
sim=np.genfromtxt(here/'onset-replay.csv',delimiter=',',names=True)
travel=np.hypot(cam['x_mm'],cam['y_mm'])
fig=plt.figure(figsize=(10,7),layout='constrained')
axes=fig.subplot_mosaic([list('abcd'),['slide']*4,['yaw']*4])
with zipfile.ZipFile(here/'onset-crops.zip') as z:
    names=sorted(z.namelist())
    for key,t in zip('abcd',[47.38,50.27,50.54,51.06]):
        i=np.argmin(abs(cam['receipt_s']-t))
        frame=cv2.imdecode(np.frombuffer(z.read(names[i]),np.uint8),cv2.IMREAD_COLOR)
        axes[key].imshow(frame[:,:,::-1]);axes[key].set_title(f"{cam['receipt_s'][i]:.3f} s");axes[key].axis('off')
for i,(key,field,target,label) in enumerate([('slide','travel_mm',travel,'Slide from start (mm)'),('yaw','yaw_deg',cam['yaw_deg'],'Yaw from start (°)')]):
    ax=axes[key]
    ax.plot(cam['receipt_s'],target,'ko-',ms=3,label='Physical glyph')
    for dy,color in [(0,'#486cad'),(5,'#bd7033')]:
        r=sim[(sim['dx_mm']==0)&(sim['dy_mm']==dy)]
        ax.plot(r['receipt_s'],r[field],'.-',color=color,label=f'Replay, Y offset {dy:+d} mm')
    ax.axvline(50.97782,ls=':',color='.35',label='SDK tap received')
    ax.set(xlabel='Seconds from undock command',ylabel=label)
    if i==0:ax.legend(fontsize=8)
fig.savefig(here/'camera-onset.png',dpi=155)
with (here/'onset-errors.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n');w.writerow(['dx_mm','dy_mm','period','n_unique_frames','slide_rmse_mm','yaw_rmse_deg'])
    for dx in (-5,0,5):
        for dy in (-5,0,5):
            r=sim[(sim['dx_mm']==dx)&(sim['dy_mm']==dy)]
            idx=np.searchsorted(cam['receipt_s'],r['receipt_s'])
            assert np.allclose(r['receipt_s'],cam['receipt_s'][idx])
            for name,lo,hi in [('early',46.5,50.3),('main',50.3,52.9)]:
                m=(r['receipt_s']>=lo)&(r['receipt_s']<=hi)
                w.writerow([dx,dy,name,sum(m),np.sqrt(np.mean((r['travel_mm'][m]-travel[idx][m])**2)),
                            np.sqrt(np.mean((r['yaw_deg'][m]-cam['yaw_deg'][idx][m])**2))])
