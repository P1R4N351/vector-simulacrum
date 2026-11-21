"""Use the wheel rims to predict the free outer fork edge on opposite flanks."""
import csv
import json
from pathlib import Path
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter1d

from fork_envelope import registered_fork
from simulacrum import params

here=Path(__file__).resolve().parent
repo=here.parents[1]
sys.path.insert(0,str(here.parent/"shoulder-photo-check/flip"))
from fitflip import world, fit

def outline(mesh,R,t,f,pp,rows,sign,dx):
    q=(mesh+[dx,0,0])@R.T+t
    uv=q[:,:,:2]/q[:,:,2:]*f+pp
    starts=uv.reshape(-1,2)
    ends=np.roll(uv,-1,axis=1).reshape(-1,2)
    result=[]
    for y in rows:
        crosses=(starts[:,1]>y)!=(ends[:,1]>y)
        a,b=starts[crosses],ends[crosses]
        xx=a[:,0]+(b[:,0]-a[:,0])*(y-a[:,1])/(b[:,1]-a[:,1])
        result.append(sign*np.max(sign*xx))
    return np.array(result)

def main():
    archive=Path(sys.argv[1])
    fork=registered_fork(repo/'web/public/robot/sdk/print/forks7.stl')
    v=params.values(params.load(model="vector_2"))
    base=np.array(v['lift_base_position_mm'])
    height=v['lift_height_min_mm']
    def mounted(h):
        return fork+base+[np.sqrt(v['lift_arm_length_mm']**2-(h-base[2])**2),0,h-base[2]]
    fig,axes=plt.subplots(1,2,figsize=(8,5),layout="constrained")
    scores=[]
    for ax,n,sign,x0,lo,hi in zip(axes,[60,195],[1,-1],[6152,1830],[4100,4060],[4440,4380]):
        photo=cv2.imread(str(archive/f"S-ring{n}-main.jpg"))
        gray=cv2.cvtColor(photo,cv2.COLOR_BGR2GRAY)
        rows=np.arange(lo,hi+1,12)
        pixels=np.arange(x0-55,x0+56)
        edge=[]
        for y in rows:
            profile=np.median(gray[y-3:y+4,pixels],axis=0)
            grad=sign*gaussian_filter1d(profile.astype(float),5,order=1)
            edge.append(pixels[np.argmax(grad)])
        edge=np.asarray(edge)
        cam=json.loads((here.parent/'shoulder-photo-check/flip'/f'flip-{n}.json').read_text())
        pp=np.asarray(cam['pp']);f=cam['f'];picks=[np.array(e) for e in cam['edges']]
        xlim=(x0-420,x0+420);ylim=(lo-370,hi+220)
        ax.imshow(cv2.cvtColor(photo[ylim[0]:ylim[1],xlim[0]:xlim[1]],cv2.COLOR_BGR2RGB),extent=(*xlim,ylim[1],ylim[0]))
        ax.plot(edge,rows,'w.',label="Photo edge")
        for variant,factor,h,radii in [
            ("nominal",1,height,[13,9.25]), ("short focal",.95,height,[13,9.25]),
            ("long focal",1.05,height,[13,9.25]), ("flat rim",1,height,[12.75,9]),
            ("lower wrist",1,height-2,[13,9.25]), ("higher wrist",1,height+2,[13,9.25])]:
            mesh=mounted(h)
            if factor==1 and variant!="flat rim":
                R,t=np.array(cam['R']),np.array(cam['t'])
            else:
                _,p=fit(picks,48,f*factor,np.array(radii))
                R,t,_=world(p,n)
            curves={dx:outline(mesh,R,t,f*factor,pp,rows,sign,dx) for dx in (-2,0,2)}
            if variant=="nominal":
                for dx,color in zip((-2,0,2),('#41dfd2','#f6b546','#de64af')):
                    ax.plot(curves[dx],rows,color=color,label=f"{dx:+} mm reach")
            for dx,pred in curves.items():
                scores.append([n,variant,factor,h,dx,round(np.mean(abs(pred-edge)),2),
                               round(np.mean(sign*(pred-edge)),2)])
        ax.set(xlim=xlim,ylim=ylim[::-1],title=f"Orbit heading {n}°",xlabel="Native image column")
    axes[0].set_ylabel("Native image row")
    axes[1].legend(fontsize=8)
    fig.savefig(here/'flank-fork.png',dpi=160)
    with (here/'flank-fork.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['heading_deg','variant','focal_scale','wrist_mm','reach_offset_mm','mean_abs_px','mean_outward_px']);w.writerows(scores)

if __name__=='__main__':
    main()
