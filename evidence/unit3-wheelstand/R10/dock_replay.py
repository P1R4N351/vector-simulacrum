"""Replay the docking impact from the two independent hub registrations."""
import sys,tempfile,csv
from itertools import product
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
current_surface='--current-surface' in sys.argv[3:]
ablation='--ablation' in sys.argv[3:]
camera_path=ablation or '--camera-path' in sys.argv[3:]
if camera_path:
 trace=np.genfromtxt(folder/'chassis-path.csv',names=True,delimiter=',')
 knots=dock+trace['dock_s']
 new_xy=xy+R.from_euler('z',heading).apply(np.column_stack([trace['opt_forward_mm'],trace['opt_left_mm'],np.zeros(len(trace))]))[:,:2]
 new_yaw=heading+np.radians(trace['opt_ccw_deg'])
 old_end=np.array([np.interp(knots[-1],poses['sec'],poses[k]) for k in ('x_mm','y_mm')])
 correction=new_yaw[-1]-np.interp(knots[-1],poses['sec'],angles)
 after=poses['sec']>knots[-1]
 delta=np.column_stack([poses[k][after]-v for k,v in zip(('x_mm','y_mm'),old_end)])
 continuation=new_xy[-1]+R.from_euler('z',correction).apply(np.column_stack([delta,np.zeros(len(delta))]))[:,:2]
 during=(poses['sec']>=knots[0]) & ~after
 for col,k in enumerate(('x_mm','y_mm')):
  poses[k][during]=np.interp(poses['sec'][during],knots,new_xy[:,col])
  poses[k][after]=continuation[:,col]
 angles[during]=np.interp(poses['sec'][during],knots,new_yaw)
 angles[after]+=correction
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
  conditions=[(),('fork',),('tread',),('carrier',),('fork','tread','carrier')] if ablation else [()]
  fixture=sys.argv[1]
  if current_surface:
   sys.path.insert(0,str(p/'R9'))
   from surface_transfer import replace_boxes
   updated=d/'current-cube.xml'
   replace_boxes(fixture,updated)
   fixture=str(updated)
  for dt,omitted in product((.0005,.00025),conditions):
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
   t,tilt,hit,speed,sensor=run(fixture,dt,True,d,R.from_euler('z',yaw),observer=observe,articulated=True,shelf_support=True,chassis_contact=True,omit_contacts=omitted)
   out=np.array(out)
   travel=np.linalg.norm(out[:,:2]-pos/1000,axis=1)*1000
   turn=np.degrees(np.unwrap(out[:,-1])-out[0,-1])
   ix=np.flatnonzero(np.isfinite(contact))
   summary.append([reg['view'],dt*1000,travel[-1],turn[-1],tilt.max(),len(sensor.taps.events),
                   t[ix[0]] if len(ix) else np.nan,contact[ix[0]]*1000 if len(ix) else np.nan,','.join(initial_pair)])
   if ablation:
    summary[-1].append('+'.join(omitted) or 'none')
   if dt==.00025:
    ax[0].plot(t+start_offset,travel,label=reg['view']+' · '+(','.join(omitted) or 'all contacts'))
    ax[1].plot(t+start_offset,turn)
with (folder/('current-surface.csv' if current_surface else 'contact-ablation.csv' if ablation else 'camera-path-replay.csv' if camera_path else 'dock-replay.csv' if start_offset==3.75 else 'dock-late-start.csv')).open('w') as f:
 w=csv.writer(f,lineterminator='\n')
 w.writerow('view dt_ms travel_mm ccw_deg peak_tilt_deg taps first_contact_from_init_s first_contact_gap_mm initial_pair'.split()+(['omitted'] if ablation else []))
 w.writerows(summary)
for a,rng in zip(ax,[(39.8,42.9),(64.8,70.9)]):
 a.axhspan(*rng,alpha=.22,color='#48865c',label='Physical endpoint estimates')
 a.axhline(0,color='.6',lw=.5)
ax[0].set_ylabel('Net displacement (mm)')
ax[1].set(xlabel='Seconds after dock command',ylabel='Cube yaw (degrees CCW)')
ax[0].legend()
if ablation:
 from ablation_plot import plot
 plot(folder)
elif start_offset==3.75:
 fig.savefig(folder/('current-surface.png' if current_surface else 'contact-ablation.png' if ablation else 'camera-path-replay.png' if camera_path else 'dock-replay.png'),dpi=150)
