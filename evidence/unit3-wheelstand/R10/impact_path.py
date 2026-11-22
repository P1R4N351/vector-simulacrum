"""Reconstruct planar motion of the R10 upper glyph on the two corrected views."""
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from hub_registration import FACES, HUB, K, SIDE, TOP, VIEWS
from tilt_frames import CENTERS, corners

HERE = Path(__file__).resolve().parent


def camera(i):
    _, offset, _, top = VIEWS[i]
    xyz = np.vstack([TOP, FACES[i]])
    uv = np.vstack([top[0], SIDE[i]])/3 + offset
    ok, rv, tv = cv2.solvePnP(xyz, uv, K, None, flags=cv2.SOLVEPNP_SQPNP)
    assert ok
    rot = cv2.Rodrigues(rv)[0]
    eye = -rot.T @ tv.ravel()

    def project(pts, z):
        rays = np.column_stack([pts, np.ones(len(pts))]) @ np.linalg.inv(K).T @ rot
        scale = (z-eye[2])/rays[:,2]
        assert np.all(scale > 0)
        return (eye+rays*scale[:,None])[:,:2]

    front,rear = project(HUB[i], np.array([14.5,10.75]))
    forward = (front-rear)/np.linalg.norm(front-rear)
    return project, np.column_stack([forward, [-forward[1],forward[0]]])


def patch(gray, inverse_h, center, yaw, radius):
    # The rectified grid is in the fixed chassis frame (forward, left).
    a = np.radians(yaw)
    rot = np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    line = np.arange(-radius,radius+.1,.5)
    grid = np.stack(np.meshgrid(line,line),axis=-1)
    world = center + grid @ rot.T
    uv = cv2.perspectiveTransform(world.reshape(-1,1,2),inverse_h).reshape(world.shape)
    return cv2.remap(gray,uv[:,:,0].astype('f'),uv[:,:,1].astype('f'),cv2.INTER_LINEAR)


def align(gray, inverse_h, template, center, predicted):
    best = (-1,None,None)
    for yaw in np.arange(predicted-25,predicted+26):
        sample = patch(gray,inverse_h,center,yaw,20)
        match = cv2.matchTemplate(sample,template,cv2.TM_CCOEFF_NORMED)
        _,score,_,loc = cv2.minMaxLoc(match)
        if score>best[0]:
            dx,dy = (np.array(loc)-(match.shape[0]-1)/2)*.5
            a = np.radians(yaw)
            shift = [dx*np.cos(a)-dy*np.sin(a), dx*np.sin(a)+dy*np.cos(a)]
            best = (score,yaw,center+shift)
    return best


def main():
    cap = cv2.VideoCapture(str(HERE/'paired.mp4'))
    rows = []
    for i,(name,centers) in enumerate(CENTERS.items()):
        project,basis = camera(i)
        uv = np.array([[150.,100.],[190.,100.],[190.,170.],[150.,170.]])
        inv_h = cv2.getPerspectiveTransform((project(uv,44)@basis).astype('f'),uv.astype('f'))
        zero = {}
        origin_centers = []
        prev_fit = 0
        previous_yaw = 0
        for j,center in enumerate(centers):
            frame = 74+j
            cap.set(cv2.CAP_PROP_POS_FRAMES,frame)
            ok,image = cap.read()
            assert ok
            gray = cv2.cvtColor(image[:,600*i:600*(i+1)],cv2.COLOR_BGR2GRAY)
            samples = []
            for threshold in range(80,191,10):
                uv = corners(gray,center,threshold)
                if uv is None:
                    continue
                xy = project(uv,44) @ basis
                edges = np.roll(xy,-1,axis=0)-xy
                angle = np.angle(np.mean(np.exp(4j*np.arctan2(edges[:,1],edges[:,0]))))/4
                if j==0:
                    origin_centers.append(xy.mean(axis=0))
                    zero[threshold] = (xy.mean(axis=0), angle)
                if threshold not in zero:
                    continue
                origin, initial_angle = zero[threshold]
                yaw = np.degrees(angle-initial_angle)
                yaw = previous_yaw+(yaw-previous_yaw+45)%90-45
                samples.append([*(xy.mean(axis=0)-origin),yaw])
            if samples:
                median = np.median(samples,axis=0)
                previous_yaw = median[2]
                lo,hi = np.percentile(samples,[10,90],axis=0)
                if j==0:
                    ref_center = np.median(origin_centers,axis=0)
                    template = patch(gray,inv_h,ref_center,0,16)
                score,prev_fit,location = align(gray,inv_h,template,ref_center+median[:2],prev_fit)
                rows.append([name,frame,frame/12-2,len(samples),*median,*lo,*hi,
                             *(location-ref_center),prev_fit,score])
    cap.release()
    with (HERE/'impact-path-camera.csv').open('w') as f:
        w = csv.writer(f,lineterminator='\n')
        w.writerow(['view','frame','dock_s','thresholds','forward_mm','left_mm','ccw_deg',
                    'forward_p10','left_p10','ccw_p10','forward_p90','left_p90','ccw_p90','fit_forward_mm','fit_left_mm','fit_ccw_deg','correlation'])
        w.writerows(rows)


def plot():
    camera_data = np.genfromtxt(HERE/'impact-path-camera.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
    fig,axes = plt.subplots(3,1,figsize=(9,7.2),sharex=True,layout='constrained')
    for ax,key,short,title in zip(axes,('forward_mm','left_mm','ccw_deg'),('forward','left','ccw'),
                                  ('Forward (mm)','Left (mm)','Yaw (degrees CCW)')):
        for order in ('original','reverse'):
            sim = np.genfromtxt(HERE/f'impact-path-{order}.csv',delimiter=',',names=True)
            ax.plot(sim['dock_s'],sim[key],label=f'Sim · {order} box order')
        with (HERE/'impact-path-candidates.csv').open() as stream:
            candidate=[r for r in csv.DictReader(stream) if r['view']=='A' and r['layout']=='2']
        ax.plot([float(r['dock_s']) for r in candidate],[float(r[key]) for r in candidate],
                '--',color='#97537c',label='Sim · A, orientation 2')
        for view,marker in [('A','o'),('B','s')]:
            obs = camera_data[camera_data['view']==view]
            ax.plot(obs['dock_s'],obs['fit_'+key],marker,ms=4,label=f'Camera {view} glyph')
            if view=='A':
                ax.plot(obs['dock_s'],obs[key],':',color='gray',label='A border only')
        ax.axhline(0,color='.6',lw=.7)
        ax.set_ylabel(title)
    axes[0].legend(ncol=2)
    axes[-1].set(xlabel='Seconds after dock command',xlim=(4.1,6.45))
    fig.savefig(HERE/'impact-path-overlay.png',dpi=150)


if __name__=='__main__':
    main()
    plot()
