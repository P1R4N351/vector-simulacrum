"""Initialize the Vector 2.0 mount from the first camera pick and score later optical travel."""
from pathlib import Path
import csv
import gzip
import io
import json
import os
import subprocess

import cv2
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
runner=here.parent/'unit3-cube-range/recorded_mount.mjs'
with gzip.open(here/'states.csv.gz','rt') as f:
    hw=np.genfromtxt(f,names=True,delimiter=',',dtype=None,encoding='utf8')
i=np.flatnonzero((hw['status']&4096)!=0)[0]
t=hw['receipt_s']-hw['receipt_s'][i]
wheel=np.c_[hw['left_mmps'],hw['right_mmps']]
start=np.flatnonzero((t< -1)&(t> -6)&(abs(wheel).max(axis=1)==0))[-1]
last=np.searchsorted(t,.60)
j=np.flatnonzero((np.rad2deg(hw['pitch_rad'][:i])>-5)&(np.rad2deg(hw['pitch_rad'][1:i+1])<=-5))[-1]+1
peak=t[j+np.argmin(hw['pitch_rad'][j:i+1])]
h=np.genfromtxt(here/'dense-hubs.csv',names=True,delimiter=',',dtype=None,encoding='utf8')
h=h[h['direction']=='backward']
manual=np.genfromtxt(here/'hubs.csv',names=True,delimiter=',')
# Established camera B registration in evidence/unit3-dock-side-registration/check.py.
world=np.array([[-23,0,71.5],[23,0,71.5],[23,0,25.5],[-23,0,25.5],[0,-94,0]],float)
picks=np.array([[855,90],[951,100],[921,218],[833,187],[668,273]],float)
K=np.array([[465,0,499.5],[0,465,185.5],[0,0,1.]])
cases=[]
cameras={}
sides={}
for count in (4,5):
    ok,rot,tr=cv2.solvePnP(world[:count],picks[:count],K,None,flags=cv2.SOLVEPNP_SQPNP)
    assert ok
    rot,tr=cv2.solvePnPRefineLM(world[:count],picks[:count],K,None,rot,tr)
    R=cv2.Rodrigues(rot)[0]
    center=(-R.T@tr).ravel()
    def inward(x,y,plane):
        rays=R.T@np.linalg.inv(K)@np.stack([x,y,np.ones_like(x)])
        return 93.205+center[1]+rays[1]*(np.sign(center[0])*plane-center[0])/rays[0]
    for plane in (27,30,33):
        name=f'a{count}p{plane}'
        out=-inward(manual['front_x'][:1],manual['front_y'][:1],plane)[0]
        # Convert the observed front axle to origin under the measured initial pitch.
        out+=14.5*np.sin(hw['pitch_rad'][start])
        mid=sum(inward(h[hub+'_x'],h[hub+'_y'],plane) for hub in ('front','rear'))/2
        cameras[name]=mid
        sides[name]=np.sign(center[0])*plane/30
        cases.append(dict(name=name,out=float(out),lat=0,yaw=0,head=float(np.rad2deg(hw['head_rad'][start])),
                          lift=float(hw['lift_mm'][start]),wheels=np.c_[t[start:last],wheel[start:last]].tolist(),
                          duration_s=float(t[last-1]-t[start])))
base=next(c for c in cases if c['name']=='a5p30')
relative_heading=float(np.rad2deg(hw['yaw_rad'][start]-hw['yaw_rad'][i]))
for lateral in (-3,0,3):
    name=f'heading_lat{lateral}'
    cameras[name]=cameras['a5p30']
    sides[name]=sides['a5p30']
    cases.append({**base,'name':name,'out':base['out']+30*sides[name]*np.sin(np.deg2rad(relative_heading)),
                  'lat':lateral,'yaw':relative_heading})
data=subprocess.check_output(['node',str(runner)],input=json.dumps(cases),text=True,env={**os.environ,'VECTOR_TRACE_DOCK_PITCH':'1','VECTOR_TRACE_DOCK_HUBS':'1'})
sim=np.genfromtxt(io.StringIO(data),names=True,delimiter=',',dtype=None,encoding='utf8')
fig,ax=plt.subplots(1,2,figsize=(10,4.3),layout='constrained')
scores=[]
for case in cases:
    name=case['name'];cm=cameras[name]
    cam=np.interp(0,h['t_s'],cm)-np.interp(peak,h['t_s'],cm)
    for variant in ('canonical','driven_rear_envelope','driven_mean_only'):
        r=sim[(sim['session']==name)&(sim['variant']==variant)]
        # Dock-relative inward coordinate of the midpoint of the visible hub faces.
        # The camera registration gives the lateral sign of the observed face.
        mid=-r['hub_center_out_mm']
        side=r['hub_side_offset_mm']
        def distance(pos):return np.interp(0,r['t_s'],pos)-np.interp(peak,r['t_s'],pos)
        visible=mid-sides[name]*side
        pred=distance(visible)
        scores.append([name,variant,case['out'],cam,pred,distance(mid),abs(distance(side)),np.any(r['on']),np.interp(0,r['t_s'],r['yaw_deg'])])
        if name=='a5p30':
            sel=(r['t_s']>=peak-.05)&(r['t_s']<=.10)
            ax[0].plot(r['t_s'][sel],visible[sel]-np.interp(peak,r['t_s'],visible),label=variant.replace('_',' '))
            ax[1].plot(r['t_s'][sel],r['pitch_deg'][sel])
    if name=='a5p30':
        keep=(h['t_s']>=peak-.05)&(h['t_s']<=.10)
        ax[0].plot(h['t_s'][keep],cm[keep]-np.interp(peak,h['t_s'],cm),'k.',label='Camera, both hubs')
        keep=(t>=peak-.05)&(t<=.10)
        ax[1].plot(t[keep],np.rad2deg(hw['pitch_rad'][keep]),'k.-',label='Physical SDK')
for a in ax:a.set_xlabel('Seconds from physical first charger flag');a.axvline(0,color='.5',ls=':')
ax[0].set_ylabel('Inward hub travel since physical pitch peak (mm)');ax[1].set_ylabel('Pitch (degrees)')
ax[0].legend(fontsize=7);ax[1].legend(fontsize=8)
fig.savefig(here/'registered-replay.png',dpi=165)
with (here/'registered-replay.csv').open('w') as f:
    w=csv.writer(f);w.writerow('registration variant start_out_mm optical_mm simulated_visible_mm simulated_center_mm nominal_side_projection_mm ever_on yaw_at_physical_flag_deg'.split());w.writerows(scores)

# Preserve centerline traces for independent observations in both cameras.
with (here/'registered-center.csv').open('w') as f:
    writer = csv.writer(f)
    writer.writerow(['variant', 't_s', 'inward_mm', 'yaw_deg'])
    for variant in ('canonical', 'driven_rear_envelope'):
        rows = sim[(sim['session']=='a5p30') & (sim['variant']==variant)]
        time = rows['t_s']
        inward = -rows['hub_center_out_mm']
        for when in np.r_[peak, time[(time>peak)&(time<0)], 0.]:
            writer.writerow([variant, when, np.interp(when,time,inward)-np.interp(peak,time,inward),
                             np.interp(when,time,rows['yaw_deg'])-np.interp(peak,time,rows['yaw_deg'])])
