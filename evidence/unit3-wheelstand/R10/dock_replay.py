"""Replay the docking impact from the two independent hub registrations."""
import sys,tempfile,csv
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
p=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(p/'R8'))
from tread_speed import run
folder=p/'R10'
s=np.genfromtxt(folder/'state.csv.gz',names=True,delimiter=',')
poses=np.genfromtxt(folder/'poses.csv.gz',names=True,delimiter=',',dtype=None,encoding='utf8')
poses=poses[poses['source']=='robot_state']
with (folder/'actions.csv').open() as f:
 dock=next(float(r['sec']) for r in csv.DictReader(f) if r['action']=='dock' and r['kind']=='begin')
registration=dock+4.25
start_offset=float(sys.argv[2]) if len(sys.argv)>2 else 3.75
zero=dock+start_offset
fig,ax=plt.subplots(2,1,figsize=(8,6),sharex=True,layout='constrained')
summary=[]
angles=np.unwrap(R.from_quat(np.column_stack([poses[k] for k in ['qx','qy','qz','qw']])).as_euler('ZYX')[:,0])
heading=np.interp(registration,poses['sec'],angles)
xy=np.array([np.interp(registration,poses['sec'],poses[k]) for k in ['x_mm','y_mm']])
for reg in csv.DictReader((folder/'hub-registration.csv').open()):
 with tempfile.TemporaryDirectory() as td:
  d=Path(td);(d/'state.csv.gz').symlink_to(folder/'state.csv.gz');(d/'actions.csv').write_text('action,sec\nroll cube,'+str(zero)+'\n')
  with (d/'roll-poses.csv').open('w') as f:
   w=csv.writer(f);w.writerow('sec source x_mm y_mm yaw_rad pitch_rad lift_mm'.split())
   pos=xy+R.from_euler('z',heading).apply([float(reg['forward_mm']),float(reg['left_mm']),0])[:2]
   yaw=heading+np.radians(float(reg['edge_yaw_mod90_deg']))
   w.writerow([-.01,'robot_observed_object',*pos,yaw,0,0])
   for r,a in zip(poses,angles):
    t=r['sec']
    if -.1<t-zero<7.05-start_offset:w.writerow([t-zero,'robot_state',r['x_mm'],r['y_mm'],a,np.interp(t,s['sec'],s['pitch']),np.interp(t,s['sec'],s['lift'])])
  for dt in (.0005,.00025):
   out=[]
   contact=[]
   initial_pair=[]
   def observe(m,data):
    b=data.body('cube')
    out.append([*b.xpos[:2],np.arctan2(b.xmat[3],b.xmat[0])])
    gaps=[c.dist for c in data.contact if b.id in m.geom_bodyid[c.geom] and 0 not in m.geom_bodyid[c.geom]]
    if not contact and gaps:
     deepest=min((c for c in data.contact if b.id in m.geom_bodyid[c.geom] and 0 not in m.geom_bodyid[c.geom]),key=lambda c:c.dist)
     initial_pair.append('/'.join(m.geom(int(k)).name for k in deepest.geom))
    contact.append(min(gaps,default=np.nan))
   t,tilt,hit,speed,sensor=run(sys.argv[1],dt,True,d,R.from_euler('z',yaw),observer=observe,articulated=True,shelf_support=True,chassis_contact=True)
   out=np.array(out)
   travel=np.linalg.norm(out[:,:2]-pos/1000,axis=1)*1000
   turn=np.degrees(np.unwrap(out[:,-1])-out[0,-1])
   ix=np.flatnonzero(np.isfinite(contact))
   summary.append([reg['view'],dt*1000,travel[-1],turn[-1],tilt.max(),len(sensor.taps.events),
                   t[ix[0]] if len(ix) else np.nan,contact[ix[0]]*1000 if len(ix) else np.nan,','.join(initial_pair)])
   if dt==.00025:
    ax[0].plot(t+start_offset,travel,label='Camera '+reg['view']+' initialization')
    ax[1].plot(t+start_offset,turn)
with (folder/('dock-replay.csv' if start_offset==3.75 else 'dock-late-start.csv')).open('w') as f:
 w=csv.writer(f,lineterminator='\n')
 w.writerow('view dt_ms travel_mm ccw_deg peak_tilt_deg taps first_contact_from_init_s first_contact_gap_mm initial_pair'.split())
 w.writerows(summary)
for a,rng in zip(ax,[(39.8,42.9),(64.8,70.9)]):
 a.axhspan(*rng,alpha=.22,color='#48865c',label='Physical endpoint estimates')
 a.axhline(0,color='.6',lw=.5)
ax[0].set_ylabel('Net displacement (mm)')
ax[1].set(xlabel='Seconds after dock command',ylabel='Cube yaw (degrees CCW)')
ax[0].legend()
if start_offset==3.75:
 fig.savefig(folder/'dock-replay.png',dpi=150)
