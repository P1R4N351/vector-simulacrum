"""Dock approach in recorded SDK time and glyph-registered camera coordinates."""
import csv
import zipfile
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from release_optics import cal, displacement, principal, scene, states, world

here = Path(__file__).resolve().parent
sim = np.genfromtxt(here/'return-replay.csv',names=True,dtype=None,encoding='utf8',delimiter=',')
release = np.genfromtxt(here/'release-optics.csv',names=True,dtype=None,encoding='utf8',delimiter=',')
# Fork tips on the raised lift in settled return frames. Outer half of each tread for depth.
tips = dict(A=(87,163,150),B=(85,160,143),C=(88,169,146),D=(79,155,145),E=(90,167,150))
wheels = dict(A=(80,173,225),B=(78,173,214),C=(78,173,214),D=(83,178,211),E=(79,173,216),F=(82,175,216))

def csvout(file, header, rows):
    with (here/file).open('w',newline='') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(header.split()); w.writerows(rows)

def read(z,key):
    return cv2.imdecode(np.frombuffer(z.read(key),np.uint8),0)

def edge(im,x,y):
    # Dark rubber to light dock, excluding the dark outer lip below.
    profile=im[y-7:y+8,x-2:x+3].mean(axis=1)
    return y-7+np.argmax(np.diff(profile))+.5

def camera_pose(name):
    x,y,r,b,c,lip=cal[name]
    pts=np.array([[x,y],[r,y],[r,b],[x,b],[c,lip]],float)
    K=np.array([[473.11,0,principal[0]],[0,473.11,principal[1]],[0,0,1.]])
    _,rot,tr=cv2.solvePnP(world,pts,K,None,flags=cv2.SOLVEPNP_SQPNP)
    rot,tr=cv2.solvePnPRefineLM(world,pts,K,None,rot,tr)
    return lambda a:cv2.projectPoints(a,rot,tr,K,None)[0].reshape(-1,2)

def tread_rims(project,depth,pitch,radius,dh):
    angle=np.linspace(0,2*np.pi,121)
    x=radius*np.cos(angle)
    z=14.5+radius*np.sin(angle)
    h=6.66+.17*(depth-29.9)+dh
    forward=x*np.cos(pitch)+z*np.sin(pitch)
    up=z*np.cos(pitch)-x*np.sin(pitch)
    return np.array([project(np.column_stack([angle*0+side,depth-93.205-forward,h+up]))
                     for side in [-23.95,23.95]])

def infer_depth(name,pixels):
    project=camera_pose(name); estimates=[]
    depths=np.linspace(0,45,91)
    for p,r,h in [(.13666,14.5,0),(.10,13.5,-2),(.18,15.5,2),(.10,15.5,-2),(.18,13.5,2)]:
        pred=np.array([tread_rims(project,d,p,r,h)[:,:,1].max(axis=1) for d in depths])
        if h==0:
            estimates.append(np.interp(np.mean(pixels),pred[::-1].mean(axis=1),depths[::-1]))
        for side in (0,1):
            for dx in (-2,2):
                estimates.append(np.interp(pixels[side]+dx,pred[::-1,side],depths[::-1]))
    return estimates[0],min(estimates),max(estimates)

def main():
    fig,axes=plt.subplots(2,3,figsize=(13,7),layout='constrained',sharex=True,sharey=True)
    qa,panels=plt.subplots(2,6,figsize=(15,5),layout='constrained')
    rows=[]; trace=[]; depthrows=[]
    with zipfile.ZipFile(here/'return-optics.zip') as z:
        for index,n in enumerate('ABCDEF'):
            ax=axes.flat[index]
            start=read(z,n+'/start.png')
            x1,x2,y=wheels[n]; pixels=[edge(start,x,y) for x in (x1,x2)]
            depth=infer_depth(n,pixels); departure=release[index]['interpolated_midpoint_mm']
            depthrows.append([n,*pixels,*depth,departure,depth[0]-departure])
            panel=panels[0,index];panel.imshow(start,cmap='gray');panel.plot([x1,x2],pixels,'r+',ms=9);panel.set_title(n+' departure')
            for depth_test,color in [(29.9,'cyan'),(16.1+departure,'orange')]:
                for rim in tread_rims(camera_pose(n),depth_test,.13666,14.5,0):
                    panel.plot(*rim.T,color=color,lw=1)
            panel.axis('off')
            if n=='F':
                ax.axis('off');ax.text(.06,.5,'F ended off the dock.\nNo return trace available.',transform=ax.transAxes);panels[1,index].axis('off');continue
            keys=sorted((float(Path(k).stem),k) for k in z.namelist() if k.startswith(n+'/') and 'start' not in k)
            t=np.array([v for v,k in keys]); imgs=[read(z,k) for v,k in keys]; base=np.argmin(abs(t-.25))
            s,i=states(n);st=np.array([float(r['receipt_s']) for r in s])
            flags=np.array([bool(int(r['status'])&4096) for r in s])
            k=next(j for j in range(i+1,len(s)) if st[j]-st[i]>3 and flags[j] and not flags[j-1])
            st-=st[k];roi=(st>=-1.1)&(st<1.2)
            assert len({r['origin'] for r,v in zip(s,roi) if v})==1
            xy=np.array([[float(r['x_mm']),float(r['y_mm'])] for r in s])
            heading=float(s[k]['yaw_rad'])
            sdk=(xy-xy[k])@np.array([np.cos(heading),np.sin(heading)])
            ax.plot(st[roi],sdk[roi],'.-',color='#007558',label='SDK odometry',ms=3)
            ax.axvspan(st[k-1],0,color='gray',alpha=.23)
            travel=scene(n);a,b,y=tips[n];pts=[(a,y),(b,y)]
            panel=panels[1,index];panel.imshow(imgs[base],cmap='gray');panel.set_title(n+' return')
            for x,y0 in pts: panel.add_patch(plt.Rectangle((x-11,y0-8),23,17,color='lime',fill=False))
            panel.axis('off')
            est=[]
            for at,im in zip(t,imgs):
                lift=[float(r['lift_mm']) for j,r in enumerate(s) if min(at,t[base])-.08<=st[j]<=max(at,t[base])+.08]
                if lift and max(lift)-min(lift)>.11:continue
                refs=[displacement(imgs[base],im,(x,y0+(11 if n=='A' else 0))) for x,y0 in [(45,107),(230,110)]]
                hits=[displacement(imgs[base],im,p) for p in pts]
                if min(sc for _,sc in refs+hits)<.80:continue
                drift=np.mean([d for d,_ in refs],axis=0)
                if not np.all(np.isfinite([d for d,_ in refs+hits])):continue
                vals=[travel(p,d-drift,h,sl) for p,(d,_) in zip(pts,hits) for h in (45,55) for sl in (0,.17)]
                est.append([at,*vals])
            e=np.array(est)
            # Zero each physical signal at its own first on-report. Camera interpolates across exposures.
            assert e[0,0]<=0<=e[-1,0]
            for col in range(1,e.shape[1]):
                e[:,col]-=np.interp(0,e[:,0],e[:,col])
            e=np.column_stack([e[:,0],e[:,1:].mean(axis=1),e[:,1:].min(axis=1),e[:,1:].max(axis=1)])
            trace.extend([[n,*row] for row in e])
            ax.fill_between(e[:,0],e[:,2],e[:,3],alpha=.15);ax.plot(e[:,0],e[:,1],'o-',ms=4,label='Camera fork tips')
            speeds=[float(s[k-1][key]) for key in ('left_mmps','right_mmps')]
            for speed in [25,35,45]:
                sr=sim[(sim['speed']==speed)&np.isin(sim['phase'],['return','seat'])]
                j=np.where(sr['on']==1)[0][0];tt=sr['t_s']-sr['t_s'][j]
                ax.plot(tt,sr['x_mm']-sr['x_mm'][j],color='#924a16',alpha=.8 if speed==35 else .25,
                        label='Sim 25/35/45 mm/s' if speed==35 else None)
                if speed==35:ax.plot(tt,sr['sdk_x_mm']-sr['sdk_x_mm'][j],'--',color='#924a16',label='Sim SDK, 35 mm/s')
            ax.set_title(n+' · first on at %.2f s'%float(s[k]['receipt_s'])); ax.set_xlim(-.95,1); ax.set_ylim(-14,39)
            with (here.parent/'unit3-physical-repeats'/n/'actions.csv').open() as f:
                dock_begin=max(float(r['receipt_s']) for r in csv.DictReader(f) if r['action']=='dock' and r['event']=='begin' and float(r['receipt_s'])<float(s[k]['receipt_s']))
            rows.append([n,s[k]['receipt_s'],float(s[k]['receipt_s'])-dock_begin,st[k-1],-sdk[k-1],*speeds,s[k]['left_mmps'],s[k]['right_mmps'],
                         float(np.max(abs(sdk[(st>=0)&(st<1)]))),e[e[:,0]<0][-1,0],e[e[:,0]>=0][0,0],
                         np.median(e[e[:,0]>=.15,1])])
    for ax in axes[-1,:]:ax.set_xlabel('Seconds from first on-contact report')
    for ax in axes[:,0]:ax.set_ylabel('Forward displacement from contact (mm)')
    axes.flat[0].legend(fontsize=8)
    fig.savefig(here/'return-optics.png',dpi=150);qa.suptitle('Top: observed lower edges (+), default start (cyan), start required by measured travel (orange) · Bottom: return fork-tip patches');qa.savefig(here/'return-landmarks.png',dpi=140)
    csvout('return-scores.csv','session first_on_s since_dock_command_s last_off_rel_s sdk_forward_step_on_mm preceding_left preceding_right first_on_left first_on_right max_sdk_first_second_mm last_camera_before_s first_camera_after_s settled_minus_on_camera_mm',rows)
    csvout('return-frames.csv','session rel_s forward_mm low_mm high_mm',trace)
    csvout('seating-depth.csv','session left_edge_px right_edge_px start_depth_mm low_mm high_mm departure_travel_mm implied_release_depth_mm',depthrows)
if __name__=='__main__':main()
