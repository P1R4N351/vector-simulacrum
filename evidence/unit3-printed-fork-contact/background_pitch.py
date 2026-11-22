"""Track distant stationary room detail through trial 1's lift."""
import csv
import sys
from pathlib import Path
import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from evaluate import cal, here, states

folder=Path(sys.argv[2])
t0=1795287608.6804-5.498106956481934
frames=sorted((float(p.name.split('-')[0])-t0,p) for p in folder.glob('*.jpg'))
reference,path=min(frames,key=lambda item:abs(item[0]-3.31))
def load(path):
    return cv2.GaussianBlur(cv2.imread(str(path),0),(5,5),1)
ref=load(path)
# Select reference corners within two distant-room regions.
mask=np.zeros_like(ref)
mask[15:105,20:650]=255
mask[105:220,20:135]=255
seeds=cv2.goodFeaturesToTrack(ref,90,.007,15,mask=mask).reshape(-1,2)
K=np.array([[cal['fx_px'],0,cal['cx_px']],[0,cal['fy_px'],cal['cy_px']],[0,0,1]])
def rays(p):
    v=np.c_[p,np.ones(len(p))]@np.linalg.inv(K).T
    return v/np.linalg.norm(v,axis=1)[:,None]
def track(img):
    src=[]; dst=[]
    for x,y in seeds.astype(int):
        r=12; s=48
        ya=max(r,y-s); xa=max(r,x-s)
        crop=img[ya-r:y+s+r+1,xa-r:x+s+r+1]
        scores=cv2.matchTemplate(crop,ref[y-r:y+r+1,x-r:x+r+1],cv2.TM_CCOEFF_NORMED)
        _,peak,_,(j,i)=cv2.minMaxLoc(scores)
        if peak>.80:
            src.append((x,y));dst.append((xa+j,ya+i))
    if len(src)<4: return np.nan,len(src),np.nan
    src,dst=np.array(src),np.array(dst)
    keep=np.linalg.norm((dst-src)-np.median(dst-src,axis=0),axis=1)<4
    if keep.sum()<4 or np.ptp(src[keep,0])<170: return np.nan,int(keep.sum()),np.nan
    rr,_=Rotation.align_vectors(rays(dst[keep]),rays(src[keep]))
    out=rr.apply(rays(src[keep]))@K.T
    err=np.sqrt(np.mean(np.sum((out[:,:2]/out[:,2,None]-dst[keep])**2,axis=1)))
    # Positive camera nose-up moves background down; apparent ray rotation is negative about camera X.
    return -np.degrees(rr.as_rotvec()[0]),int(keep.sum()),err
# Recover known signed pitch in the actual reference texture.
for pitch in (-2,2):
    H=K@Rotation.from_euler('x',-pitch,degrees=True).as_matrix()@np.linalg.inv(K)
    recovered,n,err=track(cv2.warpPerspective(ref,H,(ref.shape[1],ref.shape[0])))
    assert abs(recovered-pitch)<.15, (pitch,recovered,n,err)
r=states[states['trial']==1]
def head(t):
    return np.degrees(np.interp(t-.232+1.5,r['sec'],r['head_rad']))
data=[]
for t,p in frames:
    if 2<=t<=9:
        angle,n,err=track(load(p))
        enc=head(t)-head(reference)
        data.append((t,angle,enc,angle-enc,n,err))
with (here/'background-pitch.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('receipt_sec','camera_pitch_change_deg','head_encoder_change_deg','residual_deg','features','rms_px'))
    w.writerows(data)
t,cam,enc,res,n,err=np.array(data).T
fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
ax.plot(t,cam,'o-',label='Room features');ax.plot(t,enc,'s-',label='Head encoder')
ax.plot(t,res,'x--',label='Camera minus encoder')
ax.set(xlabel='Frame receipt, seconds after approach',ylabel='Pitch change from reference (°)')
ax.legend()
fig.savefig(here/'background-pitch.png',dpi=160)
print(np.array(data).round(2))
