"""Replay the first shelf yaw contact with measured robot kinematics and a free cube."""
import argparse
import csv
import xml.etree.ElementTree as ET
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1]))
from simulacrum import mjcf, params
from simulacrum.cube import CubeAccelerometer

out=here/'E'
robot=np.genfromtxt(out/'robot.csv.gz',delimiter=',',names=True)
observation=next(csv.DictReader((out/'object-events.csv').open()))
v=params.values(params.load(model='vector_2'))
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--printed-fixture',type=Path,help='XML from unit3-printed-fork-contact/build_fixture.py')
parser.add_argument('--export-force',type=Path,help='Save the nominal pre-filter cube acceleration for a sensor-only replay')
parser.add_argument('--cadence',action='store_true',help='Fit a uniform SDK state cadence over the 45–54 s replay interval')
args=parser.parse_args()
scene=ET.fromstring(mjcf.build_mjcf(v,(),cube='free'))
prefix='first-contact'
if args.printed_fixture:
    fixture=ET.parse(args.printed_fixture)
    fork=scene.find(".//body[@name='lift_fork']")
    original=fork.find("geom[@name='lift_fork_geom']")
    attrs={k:val for k,val in original.attrib.items() if k not in ('name','type','pos','size','mass')}
    for geom in fork.findall('geom'):
        geom.set('contype','0')
        geom.set('conaffinity','0')
    asset=ET.SubElement(scene,'asset')
    meshes=fixture.findall(".//asset/mesh")
    pieces=fixture.findall(".//body[@name='fork']/geom[@type='mesh']")
    assert len(pieces)==len(meshes)==58
    for mesh in meshes:
        asset.append(mesh)
    for geom in pieces:
        ET.SubElement(fork,'geom',dict(attrs,type='mesh',mesh=geom.attrib['mesh'],name=geom.attrib['name'],mass='0'))
    prefix='printed-contact'
model=mj.MjModel.from_xml_string(ET.tostring(scene,encoding='unicode'))
dt=model.opt.timestep
times=np.arange(45.5,53.1,dt)
rows=robot[(robot['receipt_s']>=45)&(robot['receipt_s']<=54)]
if args.cadence:
    idx=np.arange(len(rows))
    period,intercept=np.polyfit(idx,rows['receipt_s'],1)
    rows['receipt_s']=intercept+idx*period
    prefix+='-cadence'
assert np.all(rows['origin']==int(observation['origin']))
def series(key):
    return np.interp(times,rows['receipt_s'],rows[key])
def joint(name):
    return model.joint(name).qposadr[0]
root=joint('cube_root')
cube_v=model.joint('cube_root').dofadr[0]
path=np.tile(model.qpos0,(len(times),1))
x,y=series('x_mm'),series('y_mm')
heading=np.interp(times,rows['receipt_s'],np.unwrap(rows['yaw_rad']))
path[:,:2]=np.column_stack((x-x[0],y-y[0]))/1000
path[:,3]=np.cos(heading/2);path[:,6]=np.sin(heading/2)
lift=np.array([mjcf.lift_angle_of_height(v,h) for h in series('lift_mm')])
path[:,joint('lift_joint')]=lift
path[:,joint('wrist_joint')]=-lift
path[:,joint('head_joint')]=series('head_rad')
for axle,label in [('f','front'),('r','rear')]:
    for side,speed in [('l','left_mmps'),('r','right_mmps')]:
        path[:,joint(f'wheel_{axle}{side}_joint')]=np.cumsum(series(speed))*dt/v[f'wheel_{label}_radius_mm']
velocity=np.zeros((len(path),model.nv))
for i in range(len(path)-1):
    mj.mj_differentiatePos(model,velocity[i],dt,path[i],path[i+1])
center=np.array([float(observation['x_mm'])-x[0],float(observation['y_mm'])-y[0]])
# From the same native SDK observation at +6.8361 s (w,x,y,z quaternion).
w,qx,qy,qz=(-.7137432,.005623992,-.00070193043,-.70038456)
angle=np.arctan2(2*(w*qz+qx*qy),1-2*(qy*qy+qz*qz))
cube_body=model.body('cube').id
def replay(dx,dy):
    d=mj.MjData(model)
    d.qpos[root:root+2]=(center+[dx,dy])/1000
    d.qpos[root+3:root+7]=[np.cos(angle/2),0,0,np.sin(angle/2)]
    first=None;overlap=0.;trace=[]
    forces=[]
    chip=CubeAccelerometer(v,dt,seed=17)
    for i,t in enumerate(times):
        d.qpos[:root]=path[i,:root]
        d.qvel[:cube_v]=velocity[i,:cube_v]
        d.ctrl[model.actuator('head').id]=path[i,joint('head_joint')]
        d.ctrl[model.actuator('lift').id]=lift[i]
        mj.mj_step(model,d)
        force_g=d.sensor('cube_accel').data/(v['gravity_mmps2']*.001)
        if args.export_force and dx==dy==0:
            forces.append(force_g)
        chip.step((i+1)*dt,force_g)
        for c in d.contact:
            a,b=model.geom_bodyid[c.geom]
            if cube_body in (a,b) and 0 not in (a,b):
                if i==0:overlap=max(overlap,-1000*c.dist)
                if first is None:
                    other=c.geom[1] if a==cube_body else c.geom[0]
                    first=(t,model.geom(other).name)
        w1,x1,y1,z1=d.qpos[root+3:root+7]
        trace.append(np.arctan2(2*(w1*z1+x1*y1),1-2*(z1*z1+y1*y1)))
    a=np.rad2deg(np.unwrap(trace))
    def settled(lo,hi):
        return np.median(a[(times>=lo)&(times<=hi)])
    change=settled(51.78,52.58)-settled(49.38,50.18)
    packets=[p for t,p in chip.messages if abs(t+times[0]-50.97782)<1]
    peak=max(np.linalg.norm(np.frombuffer(p[2:],dtype='<i2').reshape(3,3)/8192,axis=1).max() for p in packets)
    taps=[t+times[0] for t in chip.taps.events if abs(t+times[0]-50.97782)<1]
    if dx==dy==0:
        if args.cadence or args.export_force:
            with (out/f'{prefix}-tap-times.csv').open('w') as f:
                f.write('candidate_time_s\n')
                f.writelines(f'{t:.6f}\n' for t in taps)
        if args.export_force:
            np.savez_compressed(args.export_force,forces=forces,start=times[0],dt=dt)
        observed=np.genfromtxt(out/'cube.csv.gz',delimiter=',',names=True)
        g=np.linalg.norm([observed[k] for k in ('x_counts','y_counts','z_counts')],axis=0)/8192
        fig,ax=plt.subplots(figsize=(8,3.4),layout='constrained')
        for series_t,values,label in [(observed['receipt_s'],g,'Physical cube, packet receipt'),(
                np.repeat([t+times[0] for t,p in chip.messages],3),
                np.concatenate([np.linalg.norm(np.frombuffer(p[2:],dtype='<i2').reshape(3,3)/8192,axis=1) for t,p in chip.messages]),
                'Frozen model, packet emitted')]:
            mask=np.abs(series_t-50.97782)<1
            ax.plot(series_t[mask],values[mask],'.-',ms=2,lw=.8,label=label)
        ax.set(xlabel='Seconds from undock command',ylabel='Specific force magnitude (g)',title='First shelf contact at the SDK cube position')
        ax.legend()
        fig.savefig(out/f'{prefix}-force.png',dpi=150)
        plt.close(fig)
    return overlap,*(first or (None,'')),change,peak,len(taps)
results=[]
for dy in (-5,0,5):
    for dx in (-5,0,5):
        results.append([dx,dy,*replay(dx,dy)])
with (out/f'{prefix}-replay.csv').open('w') as f:
    writer=csv.writer(f,lineterminator='\n')
    writer.writerow(['cube_dx_mm','cube_dy_mm','initial_overlap_mm','first_contact_s','first_geom','yaw_change_deg','packet_peak_g','packet_taps_in_window'])
    writer.writerows(results)
fig,ax=plt.subplots(figsize=(8,4.5),layout='constrained')
ax.axhspan(14.148,18.013,color='#d3e5d7',label='Two optical estimates')
for dy in (-5,0,5):
    group=[r for r in results if r[1]==dy]
    ax.plot([r[0] for r in group],[r[5] for r in group],'.-',label=f'Cube y offset {dy:+d} mm')
ax.set(xlabel='Cube x offset from SDK sighting (mm)',ylabel='Cube yaw change (degrees)',xticks=[-5,0,5],title='First shelf contact · measured robot trajectory')
ax.legend()
fig.savefig(out/f'{prefix}-replay.png',dpi=150)
if args.printed_fixture and not args.cadence:
    baseline=np.genfromtxt(out/'first-contact-replay.csv',names=True,delimiter=',',dtype=None,encoding='utf8')
    assert np.array_equal(list(zip(baseline['cube_dx_mm'],baseline['cube_dy_mm'])),[r[:2] for r in results])
    fig,axes=plt.subplots(1,3,figsize=(10,3.8),layout='constrained')
    for ax,column,key,target,title in zip(axes,(5,6,7),('yaw_change_deg','packet_peak_g','packet_taps_in_window'),
                                           ((14.148,18.013),(1.078,1.078),(1,1)),
                                           ('Cube yaw (°)','Peak specific force (g)','Packet tap count')):
        for idx,r in enumerate(results):
            ax.plot([0,1],[baseline[key][idx],r[column]],'o-',color='#284971' if idx==4 else '.7',
                    lw=2 if idx==4 else .8,zorder=4 if idx==4 else 1,ms=5 if idx==4 else 3)
        if target[0]==target[1]:ax.axhline(target[0],color='#367843',ls='--')
        else:ax.axhspan(*target,color='#cee7d0')
        ax.set(xticks=[0,1],xticklabels=['Three boxes','Printed fork'],title=title,xlim=(-.2,1.2))
    fig.suptitle('Same shelf contact and trajectory · blue: SDK cube pose · gray: ±5 mm grid · green: physical')
    fig.savefig(out/'fork-transfer.png',dpi=150)
print(results)
