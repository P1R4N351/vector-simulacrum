"""Replay the docking impact from the two independent hub registrations."""
import sys,tempfile,csv
import xml.etree.ElementTree as ET
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
early_path="--early-path" in sys.argv[3:]
early_rows=[]
fixed_check="--fixed-shoulder" in sys.argv[3:]
if fixed_check and not (early_path and "--face-test" in sys.argv[3:]):
 raise ValueError("fixed shoulder requires --face-test --early-path")
lift_check="--lift-check" in sys.argv[3:]
if lift_check:
 from lift_check import LiftCheck
 lift=LiftCheck(folder)
 if not early_path or "--face-test" not in sys.argv[3:]:
  raise ValueError("lift check requires --face-test --early-path")
inertia_test="--inertia-test" in sys.argv[3:]
com_test="--com-test" in sys.argv[3:]
mass_test=inertia_test or com_test
if inertia_test and com_test:
 raise ValueError("vary one mass property per test")
if mass_test and not (early_path and "--face-test" in sys.argv[3:]):
 raise ValueError("mass test requires --face-test --early-path")
onset_rows=[]
pose_grid="--pose-grid" in sys.argv[3:]
if pose_grid and not early_path:
 raise ValueError("pose grid requires --early-path --face-test")
impulse_test="--impact-impulse" in sys.argv[3:]
if impulse_test and not early_path:
 raise ValueError("impact impulse requires --face-test --early-path")

angles=np.unwrap(R.from_quat(np.column_stack([poses[k] for k in ['qx','qy','qz','qw']])).as_euler('ZYX')[:,0])
heading=np.interp(registration,poses['sec'],angles)
xy=np.array([np.interp(registration,poses['sec'],poses[k]) for k in ['x_mm','y_mm']])
face_test='--face-test' in sys.argv[3:]
if early_path and not face_test:
 raise ValueError('early path comparison requires --face-test')
reverse_boxes='--reverse-box-order' in sys.argv[3:]
if reverse_boxes and not face_test:
 raise ValueError('box order comparison requires --face-test')
face_output='face-replay-reverse' if reverse_boxes else 'face-replay'
solver_check='--solver-check' in sys.argv[3:]
tight_solver='--tight-solver' in sys.argv[3:]
if solver_check:
 if not face_test:
  raise ValueError('solver comparison requires --face-test')
 face_output='solver-'+('tight' if tight_solver else 'default')+('-reverse' if reverse_boxes else '-original')
current_surface=face_test or '--current-surface' in sys.argv[3:]
ablation='--ablation' in sys.argv[3:]
camera_path=ablation or face_test or '--camera-path' in sys.argv[3:]
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
    if -.1<t-zero<(5.0 if early_path and not (pose_grid or mass_test) else 7.05)-start_offset:w.writerow([t-zero,'robot_state',r['x_mm'],r['y_mm'],a,np.interp(t,s['sec'],s['pitch']),np.interp(t,s['sec'],s['lift'])])
  conditions=[(),('fork',),('tread',),('carrier',),('fork','tread','carrier')] if ablation else [()]
  fixture=sys.argv[1]
  if current_surface:
   sys.path.insert(0,str(p/'R9'))
   from surface_transfer import replace_boxes
   updated=d/'current-cube.xml'
   replace_boxes(fixture,updated)
   if reverse_boxes:
    tree=ET.parse(updated)
    body=tree.find(".//body[@name='cube']")
    geoms=body.findall('geom')
    for geom in geoms:
     body.remove(geom)
    body.extend(reversed(geoms))
    tree.write(updated)
   if solver_check and tight_solver:
    tree=ET.parse(updated)
    tree.find('option').attrib.update(iterations='300', tolerance='1e-12', ls_iterations='100', ls_tolerance='1e-6')
    tree.write(updated)
   fixture=str(updated)
  layouts=[('upright',R.identity())]
  if face_test:
   with (folder/'rigid-faces.csv').open() as f:
    for rank,row in enumerate(list(csv.DictReader(f))[:4],1):
     matrix=np.array([int(row[f'r{i}{j}']) for i in range(3) for j in range(3)]).reshape(3,3)
     layouts.append((str(rank),R.from_matrix(matrix)))
  timesteps=(.00025,) if reverse_boxes else (.0005,.00025)
  if early_path:
   timesteps=(.00025,)
  if solver_check:
   timesteps=(.00025,.000125)
   layouts=[item for item in layouts if item[0]=='4' or (reg['view']=='B' and item[0]=='1')]
  if solver_check and tight_solver and reg['view']=='B':
   timesteps=(*timesteps,.0000625)
  offsets=list(product((-2,0,2),(-3,0,3))) if pose_grid else [(0,0)]
  if pose_grid:
   layouts=[item for item in layouts if item[0]=='2']
  if mass_test or lift_check or fixed_check:
   layouts=[item for item in layouts if item[0]=='2']
  multipliers=(.5,1.,5/3) if inertia_test else (1.,)
  centers=[(0,0,0)]
  if com_test:
   centers.extend(tuple(sign*3 if i==axis else 0 for i in range(3)) for axis in range(3) for sign in (-1,1))
  gains=(1,4,16) if lift_check else (1,)
  shoulders=(None,float(np.interp(registration,s["sec"],s["lift"]))) if fixed_check else (None,)
  for dt,omitted,(layout,asset_rotation),(df,dl),multiplier,center,gain,shoulder in product(timesteps,conditions,layouts,offsets,multipliers,centers,gains,shoulders):
   replay_fixture=fixture
   if multiplier!=1. or any(center):
    tree=ET.parse(fixture)
    inertial=tree.find(".//body[@name='cube']/inertial")
    diag=np.fromstring(inertial.get('diaginertia'),sep=' ')
    inertial.set('diaginertia',' '.join(str(val*multiplier) for val in diag))
    inertial.set('pos',' '.join(str(val*.001) for val in center))
    replay_fixture=str(d/'mass.xml')
    tree.write(replay_fixture)
   if dt==.0000625 and layout!='4':
    continue
   if impulse_test:
    if layout!='2':
     continue
    from impact_wrench import ImpactWrench
    wrench_recorder=ImpactWrench()
   tilt_trace='--tilt-trace' in sys.argv[3:]
   if tilt_trace and not (solver_check and tight_solver and reg['view']=='B' and layout=='4' and dt==.0000625):
    continue
   out=[]
   contact=[]
   initial_pair=[]
   first_patch=[]
   iterations=[]
   if lift_check:
    lift.begin()
   def observe(m,data):
    if lift_check:
     lift.sample(data)
    if impulse_test:
     wrench_recorder.sample(m,data)
    b=data.body('cube')
    if solver_check:
     iterations.append(max(data.solver_niter))
    out.append([*b.xpos[:2],np.arctan2(b.xmat[3],b.xmat[0])])
    if face_test:
     relative=b.xmat.reshape(3,3)@initial.as_matrix().T
     out[-1][-1]=np.arctan2(relative[1,0]-relative[0,1],relative[0,0]+relative[1,1])
     if not first_patch:
      for c in data.contact:
       bodies=m.geom_bodyid[c.geom]
       if b.id in bodies and m.body('fork').id in bodies:
        point=(c.pos-b.xpos)@b.xmat.reshape(3,3)*1000
        first_patch.extend([m.geom(int(c.geom[np.argmax(bodies==b.id)])).name,*point])
        break
    gaps=[c.dist for c in data.contact if b.id in m.geom_bodyid[c.geom] and 0 not in m.geom_bodyid[c.geom]]
    if not initial_pair and gaps:
     deepest=min((c for c in data.contact if b.id in m.geom_bodyid[c.geom] and 0 not in m.geom_bodyid[c.geom]),key=lambda c:c.dist)
     initial_pair.append('/'.join(m.geom(int(k)).name for k in deepest.geom))
    contact.append(min(gaps,default=np.nan))
   initial=R.from_euler('z',yaw)
   if layout!='upright':
    initial=initial*asset_rotation
   t,tilt,hit,speed,sensor=run(replay_fixture,dt,True,d,initial,fixed_shoulder_mm=shoulder,sensor_overrides=lift.override(gain) if lift_check else None,observer=observe,articulated=True,shelf_support=True,chassis_contact=True,omit_contacts=omitted,
       initial_xy_mm=pos+R.from_euler("z",heading).apply([df,dl,0])[:2] if pose_grid else None)
   if lift_check:
    lift.finish(reg["view"],gain,start_offset,dt,np.array(out),s,dock)
    continue
   if impulse_test:
    wrench_recorder.save(reg['view'],start_offset,dt)
    path=np.asarray(out)[:,:2]
    distance=np.linalg.norm(path-path[0],axis=1)*1000
    for level in (1,2,4):
     indices=np.flatnonzero(distance>=level)
     if len(indices):
      j=indices[0]
      onset_rows.append([reg['view'],level,t[j-1]+start_offset,t[j]+start_offset])

    continue
   if tilt_trace or early_path:
    samples=np.asarray(out)
    offset_xy=(samples[:,:2]-samples[0,:2])*1000
    yaw_trace=np.degrees(np.unwrap(samples[:,2])-samples[0,2])
    stride=round(.001/dt)
    trajectory='impact-path-'+('reverse' if reverse_boxes else 'original')+'.csv'
    xy_body=R.from_euler('z',-heading).apply(np.column_stack([offset_xy,np.zeros(len(t))]))[:,:2]
    if early_path:
     for frame in range(74,102 if pose_grid or mass_test else 83 if fixed_check else 81):
      clock=frame/12-2
      vals=[np.interp(clock,t+start_offset,v) for v in (*xy_body.T,yaw_trace)]
      early_rows.append([reg['view'],layout,frame,clock,*vals]+([df,dl] if pose_grid else [])+([multiplier] if inertia_test else list(center) if com_test else [int(shoulder is not None)] if fixed_check else []))
     continue
    np.savetxt(folder/trajectory,np.column_stack([t+start_offset,xy_body,yaw_trace])[::stride],
               delimiter=',',header='dock_s,forward_mm,left_mm,ccw_deg',comments='',fmt='%.6f')
    # Save the control-tick trace at 1 ms for plotting against video at 12 fps.
    stride=round(.001/dt)
    name='tilt-reverse.csv' if reverse_boxes else 'tilt-original.csv'
    np.savetxt(folder/name,np.column_stack([t+start_offset,tilt])[::stride],
               delimiter=',',header='dock_s,tilt_deg',comments='',fmt='%.6f')
    continue
   out=np.array(out)
   travel=np.linalg.norm(out[:,:2]-pos/1000,axis=1)*1000
   turn=np.degrees(np.unwrap(out[:,-1])-out[0,-1])
   ix=np.flatnonzero(np.isfinite(contact))
   summary.append([reg['view'],dt*1000,travel[-1],turn[-1],tilt.max(),len(sensor.taps.events),
                   t[ix[0]] if len(ix) else np.nan,contact[ix[0]]*1000 if len(ix) else np.nan,','.join(initial_pair)])
   if face_test:
    summary[-1].extend([layout,*(first_patch or ['',np.nan,np.nan,np.nan])])
   if solver_check:
    summary[-1].extend([max(iterations), np.mean(iterations)])
   if ablation:
    summary[-1].append('+'.join(omitted) or 'none')
   if dt==.00025:
    ax[0].plot(t+start_offset,travel,label=reg['view']+' · '+(layout if face_test else ','.join(omitted) or 'all contacts'))
    ax[1].plot(t+start_offset,turn)
if lift_check:
 lift.save()
 sys.exit(0)
if impulse_test:
 with (folder/'sim-onset.csv').open('w') as f:
  w=csv.writer(f,lineterminator='\n')
  w.writerow(['registration','displacement_mm','last_below_s','first_above_s'])
  w.writerows(onset_rows)
 sys.exit(0)
if early_path:
 with (folder/('fixed-path.csv' if fixed_check else 'com-path.csv' if com_test else 'inertia-path.csv' if inertia_test else 'pose-grid.csv' if pose_grid else 'impact-path-candidates.csv')).open('w') as f:
  w=csv.writer(f,lineterminator='\n')
  w.writerow('view layout frame dock_s forward_mm left_mm ccw_deg'.split()+(['offset_forward_mm','offset_left_mm'] if pose_grid else [])+(['inertia_multiplier'] if inertia_test else 'com_x_mm com_y_mm com_z_mm'.split() if com_test else ['fixed_shoulder'] if fixed_check else []))
  w.writerows(early_rows)
 sys.exit(0)
if '--tilt-trace' in sys.argv[3:]:
 if not (solver_check and tight_solver):
  raise ValueError('tilt trace needs --face-test --solver-check --tight-solver')
 sys.exit(0)
with (folder/(face_output+'.csv' if face_test else 'current-surface.csv' if current_surface else 'contact-ablation.csv' if ablation else 'camera-path-replay.csv' if camera_path else 'dock-replay.csv' if start_offset==3.75 else 'dock-late-start.csv')).open('w') as f:
 w=csv.writer(f,lineterminator='\n')
 w.writerow('view dt_ms travel_mm ccw_deg peak_tilt_deg taps first_contact_from_init_s first_contact_gap_mm initial_pair'.split()+(['omitted'] if ablation else [])+(['layout','first_cube_patch','patch_x_mm','patch_y_mm','patch_z_mm'] if face_test else [])+(['max_solver_iterations','mean_solver_iterations'] if solver_check else []))
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
elif start_offset==3.75 and not solver_check:
 fig.savefig(folder/(face_output+'.png' if face_test else 'current-surface.png' if current_surface else 'contact-ablation.png' if ablation else 'camera-path-replay.png' if camera_path else 'dock-replay.png'),dpi=150)
