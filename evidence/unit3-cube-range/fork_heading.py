"""Estimate physical yaw from two bilateral fork features in rectified dock-return images."""
import csv
import zipfile
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from release_optics import cal, principal, states, world
from return_optics import read

here=Path(__file__).resolve().parent
# Per image: left and right inner tip highlights, then the lower pair of round highlights.
seeds={
 'A':[(86,148),(157,151),(83,168),(154,172)],
 'B':[(90,146),(160,148),(87,167),(159,169)],
 'C':[(95,146),(165,146),(93,166),(165,166)],
 'D':[(79,145),(149,150),(74,165),(145,170)],
 'E':[(92,157),(164,158),(90,178),(163,179)]}

def centroid(image,p):
    x,y=p
    roi=image[y-4:y+5,x-4:x+5].astype(float)
    weights=np.maximum(0,roi-(roi.min()+.55*np.ptp(roi)))
    yy,xx=np.indices(roi.shape)
    return np.array([x-4+(weights*xx).sum()/weights.sum(),y-4+(weights*yy).sum()/weights.sum()])

def references(name,dock):
    x,y,r,b,c,lip=cal[name]
    manual=np.array([[x,y],[r,y],[r,b],[x,b],[c,lip]],float)
    # Fit outer edges of the white square on their straight segments, omitting rounded corners.
    def line(im,lo,hi,guess,direction):
        along=np.arange(lo+12,hi-11)
        grad=direction*np.diff(im[guess-5:guess+6,along].astype(float),axis=0)
        at=guess-4.5+np.argmax(grad,axis=0)
        return np.polyfit(along,at,1)
    top=line(dock,x,r,y,1);bottom=line(dock,x,r,b,-1)
    left=line(dock.T,y,b,x,1);right=line(dock.T,y,b,r,-1)
    def cross(h,v):
        u=(v[0]*h[1]+v[1])/(1-v[0]*h[0])
        return [u,np.polyval(h,u)]
    return manual,np.array([cross(top,left),cross(top,right),cross(bottom,right),cross(bottom,left),[c,lip]])

def rays(points,focal,uv):
    K=np.array([[focal,0,principal[0]],[0,focal,principal[1]],[0,0,1.]])
    _,rot,tr=cv2.solvePnP(world,points,K,None,flags=cv2.SOLVEPNP_SQPNP)
    rot,tr=cv2.solvePnPRefineLM(world,points,K,None,rot,tr)
    R=cv2.Rodrigues(rot)[0]
    return -R.T@tr.ravel(),np.linalg.solve(K,np.c_[uv,np.ones(len(uv))].T).T@R

def yaw(origin,rays,height,dz):
    heights=height+np.array([-dz/2,dz/2])
    pts=origin+rays*((heights-origin[2])/rays[:,2])[:,None]
    dx,dy=np.diff(pts,axis=0)[0,:2]
    return np.rad2deg(np.arctan2(dy,dx))

def main():
    fig,axes=plt.subplots(2,5,figsize=(13,5),layout='constrained')
    scores=[]
    targets=list(csv.DictReader((here/'mount-target-inputs.csv').read_text().splitlines()))
    for col,obs in enumerate(targets):
        n=obs['session']
        with zipfile.ZipFile(here/'return-optics.zip') as z:
            at,key=max((float(Path(k).stem),k) for k in z.namelist() if k.startswith(n+'/-'))
            im=read(z,key)
        with zipfile.ZipFile(here/'release-optics.zip') as z:
            dock=read(z,n+'/dock.png')
        manual,fitted=references(n,dock)
        uv=np.array([centroid(im,p) for p in seeds[n]])
        s,i=states(n);t=np.array([float(r['receipt_s']) for r in s])
        on=np.array([int(bool(int(r['status'])&4096)) for r in s])
        k=np.flatnonzero((np.diff(on)==1)&(t[1:]>t[i]+3))[0]+1
        t-=t[k]
        # Use latest SDK sample at or before camera receipt (C's pose resets just afterwards).
        j=np.searchsorted(t,at,side='right')-1
        a,b,c,d=[float(obs['q'+str(q)]) for q in range(4)]
        out=np.arctan2(-2*(a*d+b*c),-(1-2*(c*c+d*d)))
        err=float(s[j]['yaw_rad'])-out;sdk=np.rad2deg(np.arctan2(np.sin(err),np.cos(err)))
        estimates=[]
        for pair,height in [(uv[:2],50),(uv[2:],40)]:
            origin,directions=rays(fitted,473.11,pair)
            nominal=yaw(origin,directions,height,0)
            variants=[]
            calibrations = [manual, fitted]
            for skew in (-1, 1):
                for lip_shift in (-3, 3):
                    changed = fitted.copy()
                    changed[:, 1] += np.array([-skew, skew, skew, -skew, lip_shift])
                    calibrations.append(changed)
            for calib in calibrations:
                for focal in [450,473.11,500]:
                    for bias in [-1,0,1]:
                        u=pair.copy();u[:,1]+=[-bias/2,bias/2]
                        o,dirs=rays(calib,focal,u)
                        variants.extend(yaw(o,dirs,h,z) for h in [height-5,height+5] for z in [-1,0,1])
            estimates.append((nominal,min(variants),max(variants)))
        scores.append([n,at,t[j],sdk,*np.array(estimates).ravel()])
        ax=axes[0,col];ax.imshow(im,cmap='gray');ax.set(xlim=(55,195),ylim=(200,118),title=f'{n} · {at:+.3f} s')
        for pts,color in [(uv[:2],'#ffbd53'),(uv[2:],'#00d4da')]:ax.plot(*pts.T,'+-',color=color)
        ax.axis('off')
        ax=axes[1,col]
        for x,(v,lo,hi),color in zip([0,1],estimates,['#b96b16','#008989']):
            ax.vlines(x,lo,hi,color=color);ax.scatter(x,v,c=color)
        ax.scatter(2,sdk,marker='x',color='#353666');ax.axhline(0,c='gray',lw=.7)
        ax.set(xticks=[0,1,2],xticklabels=['Tips','Lower','SDK'],xlim=(-.5,2.5));ax.set_ylim(-30,22)
    axes[1,0].set_ylabel('Yaw relative to dock (°)')
    fig.savefig(here/'fork-heading.png',dpi=160)
    with (here/'fork-heading.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow('session camera_s sdk_s sdk_deg tip_deg tip_low tip_high lower_deg lower_low lower_high'.split());w.writerows(scores)
if __name__=='__main__':main()
