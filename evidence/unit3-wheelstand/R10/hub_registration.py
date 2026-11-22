"""Locate the pre-impact cube in the chassis frame from each opposed view."""
import csv
import cv2
import numpy as np
from planar_slide import HERE, K, VIEWS
import matplotlib.pyplot as plt

# Glyph corners, clockwise from image upper left, in the enlarged crops used
# for the endpoint picks. B uses the brighter right-hand vertical face.
SIDE = np.array([[[238,241],[375,259],[367,377],[231,352]],
                 [[332,279],[391,249],[400,345],[341,383]]])
HUB = np.array([[[370,176],[304,182]],[[238,201],[330,195]]],float)
TOP = np.array([[-11.5,11.5,44],[11.5,11.5,44],[11.5,-11.5,44],[-11.5,-11.5,44]])
FACES = [
    [[-11.5,-22,33.5],[11.5,-22,33.5],[11.5,-22,10.5],[-11.5,-22,10.5]],
    [[22,-11.5,33.5],[22,11.5,33.5],[22,11.5,10.5],[22,-11.5,10.5]],
]


def locate(view, uv, hubs, extension=28):
    xyz=np.vstack([TOP,FACES[view]])
    ok,r,t=cv2.solvePnP(xyz,uv,K,None,flags=cv2.SOLVEPNP_SQPNP)
    assert ok
    rotation=cv2.Rodrigues(r)[0]
    camera=-rotation.T@t.ravel()
    ray=(np.column_stack([hubs,np.ones(2)])@np.linalg.inv(K).T)@rotation
    scale=(np.array([14.5,10.75])-camera[2])/ray[:,2]
    assert np.all(scale>0)
    front,rear=(camera+scale[:,None]*ray)[:,:2]
    span=np.linalg.norm(front-rear)
    forward=(front-rear)/span
    left=np.array([-forward[1],forward[0]])
    middle=front + (1 if view==0 else -1)*extension*left
    predicted=cv2.projectPoints(xyz,r,t,K,None)[0][:,0]
    return np.array([-middle@forward,-middle@left,span,np.sqrt(np.mean((predicted-uv)**2)), (np.degrees(np.arctan2(left[0],forward[0]))+45)%90-45])


def main():
    cap=cv2.VideoCapture(str(HERE/'paired.mp4'))
    cap.set(cv2.CAP_PROP_POS_FRAMES,75)
    ok,frame=cap.read()
    assert ok
    cap.release()
    rng=np.random.default_rng(10)
    fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
    with (HERE/'hub-registration.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['view','forward_mm','left_mm','recovered_axle_span_mm','glyph_rms_px','edge_yaw_mod90_deg','forward_p05','forward_p95','left_p05','left_p95'])
        for i,(name,offset,_,corners) in enumerate(VIEWS):
            uv=np.vstack([corners[0],SIDE[i]])/3+offset
            result=locate(i,uv,HUB[i])
            samples=[locate(i,uv+rng.normal(0,.75,uv.shape),HUB[i]+rng.normal(0,1,HUB[i].shape),
                            rng.uniform(28,30)) for _ in range(200)]
            limits=np.percentile(np.array(samples)[:,:2],[5,95],axis=0).T.ravel()
            row=[name,*result,*limits]
            print(row); w.writerow(row)
            ax=axes[i]
            ax.imshow(frame[:,600*i:600*(i+1),::-1])
            for points in np.split(uv,2):
                ax.plot(*points[[0,1,2,3,0]].T,color='cyan',lw=.8)
            ax.plot(*HUB[i].T,'x-',color='orange')
            ax.set(xlim=(min(uv[:,0].min(),HUB[i][:,0].min())-30,max(uv[:,0].max(),HUB[i][:,0].max())+30),ylim=(240,75),title=f'Camera {name} · +4.25 s')
            ax.axis('off')
    fig.savefig(HERE/'hub-registration.png',dpi=150)


if __name__=='__main__':
    main()
