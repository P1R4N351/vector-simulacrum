"""W4 finite-actuator replay. Supply XML from the printed-fork fixture builder."""
import csv
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[2]))
from simulacrum import mjcf,params
from simulacrum.cube import CubeAccelerometer
from simulacrum.drive import TreadController

held_out="--w8" in sys.argv[2:]
if held_out and '--firmware' not in sys.argv[2:]:
    raise ValueError('Use --firmware for the W8 transfer test')
if held_out:
    here=here.parent/"W8"
v=params.values(params.load(model='vector_2'))
tree=ET.fromstring(mjcf.build_mjcf(v,(),cube='free'))
fixture=ET.parse(sys.argv[1])
asset=ET.SubElement(tree,'asset')
asset.extend(fixture.findall('.//asset/mesh'))
fork=tree.find(".//body[@name='lift_fork']")
attrs={k:x for k,x in fork.find('geom').attrib.items() if k not in ('name','type','size','pos','mass')}
for geom in fork.findall('geom'):
    geom.set('contype','0');geom.set('conaffinity','0')
pieces=fixture.findall(".//body[@name='fork']/geom[@type='mesh']")
assert len(pieces)==58
for piece in pieces:
    ET.SubElement(fork,'geom',dict(attrs,type='mesh',mesh=piece.get('mesh'),name=piece.get('name'),mass='0'))
m=mj.MjModel.from_xml_string(ET.tostring(tree,encoding='unicode'))
sensor_dt=v['sim_timestep_ms']*.001
dt=sensor_dt/4
m.opt.timestep=dt
approach=np.genfromtxt(here/'approach.csv',names=True,delimiter=',')
state=np.genfromtxt(here/'state.csv.gz',names=True,delimiter=',')
with (here/'actions.csv').open() as f:
    zero=next(float(r['sec']) for r in csv.DictReader(f) if r['action']=='pop_a_wheelie')
# Time relative to the last raised-lift observation; the internal action timestamp is unavailable.
late_onset=4.015283
early_onset=3.950686
if held_out:
    drop=np.flatnonzero((approach['lift_mm'][1:]-approach['lift_mm'][:-1]) < -10)[0]+1
    early_onset,late_onset=approach['s'][drop-2:drop]
shift=late_onset-4.015283
target=json.loads((here/'initial-cube.json').read_text())
w,x,y,z=[target['q'+str(i)] for i in range(4)]
angle=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
cube_id=m.body('cube').id
cube=m.joint('cube_root').qposadr[0]
def obs(t,k):
    return np.interp(t,approach['s'],approach[k]) if k in approach.dtype.names else np.interp(t+zero,state['sec'],state[k])
def lowering_goal(elapsed, start_angle):
    # Symmetric velocity-limited profile, from rest, using the stock action bounds.
    goal=mjcf.lift_angle_of_height(v,32)
    distance=max(0,start_angle-goal)
    vmax=min(10.,np.sqrt(200.*distance))
    ramp=vmax/200.
    if vmax==0:return goal
    cruise=max(0.,distance/vmax-ramp)
    end=2*ramp+cruise
    if elapsed<ramp:return start_angle-100.*elapsed**2
    if elapsed<ramp+cruise:return start_angle-.5*vmax*ramp-vmax*(elapsed-ramp)
    return goal+100.*max(0.,end-elapsed)**2

def run(start,limit,onset=None):
    d=mj.MjData(m)
    m.actuator('lift').forcerange[:]=[-limit*.001,limit*.001]
    yaw=obs(start,'yaw')
    d.qpos[:2]=np.array([obs(start,'x_mm'),obs(start,'y_mm')])*.001
    d.qpos[3:7]=[np.cos(yaw/2),0,0,np.sin(yaw/2)]
    d.qpos[cube:cube+2]=np.array([target['x'],target['y']])*.001
    d.qpos[cube+3:cube+7]=[np.cos(angle/2),0,0,np.sin(angle/2)]
    lift=mjcf.lift_angle_of_height(v,obs(start,'lift_mm'))
    for name,val in [('lift',lift),('wrist',-lift),('head',obs(start,'head'))]:
        d.joint(name+'_joint').qpos[0]=val
    d.ctrl[m.actuator('head').id]=obs(start,'head')
    d.ctrl[m.actuator('lift').id]=lift
    if held_out:
        mj.mj_forward(m,d)
        depths=[-contact.dist*1000 for contact in d.contact
                if cube_id in m.geom_bodyid[contact.geom] and contact.dist<0
                and 0 not in m.geom_bodyid[contact.geom]]
        initial_xy=d.qpos[cube:cube+2].copy()
    for _ in range(round(.20/dt)):mj.mj_step(m,d)
    if held_out:
        initial_checks.append([start,onset,max(depths,default=0.),
                               np.linalg.norm(d.qpos[cube:cube+2]-initial_xy)*1000])
    center=d.qpos[cube:cube+2].copy()
    speed=np.mean([obs(start,'left'),obs(start,'right')])
    d.qvel[:2]=.001*speed*np.array([np.cos(yaw),np.sin(yaw)])
    for axle,radius in [('f',v['wheel_front_radius_mm']),('r',v['wheel_rear_radius_mm'])]:
        for side,key in [('l','left'),('r','right')]:
            d.joint('wheel_'+axle+side+'_joint').qvel[0]=obs(start,key)/radius
    controllers=[TreadController(v),TreadController(v)]
    sensor=CubeAccelerometer(v,sensor_dt,seed=17)
    trace=[];peak_load=0
    drive_ended=False
    lowering_start=mjcf.lift_angle_of_height(v,obs(onset,"lift_mm")) if onset is not None else lift
    for i,t in enumerate(np.arange(start,5.8+shift,dt)):
        if i%round(.001*v['control_period_ms']/dt)==0:
            lowering=onset is not None and t>=onset
            if lowering:
                chassis=m.jnt_bodyid[m.joint('root').id]
                pitch_rad=np.arcsin(np.clip(d.xmat[chassis].reshape(3,3)[2,0],-1,1))
                drive_ended |= pitch_rad>1.2 or t-onset>.5
            for controller,side in zip(controllers,['left','right']):
                measured=d.joint('wheel_f'+side[0]+'_joint').qvel[0]*v['wheel_front_radius_mm']
                requested=obs(t,side) if t<4.84+shift else 0
                if lowering:requested=0 if drive_ended else 150
                duty=controller.update(requested,measured)
                d.actuator(side+'_wheel').ctrl[0]=duty*3.92/v['motor_reference_voltage_v']
            d.actuator('lift').ctrl[0]=(lowering_goal(t-onset,lowering_start) if lowering
                                           else mjcf.lift_angle_of_height(v,obs(t,'lift_mm')))
            d.actuator('head').ctrl[0]=obs(t,'head')
        mj.mj_step(m,d)
        downward=0.
        for j,c in enumerate(d.contact):
            a,b=m.geom_bodyid[c.geom]
            if cube_id in (a,b) and m.body('lift_fork').id in (a,b):
                wrench=np.zeros(6);mj.mj_contactForce(m,d,j,wrench)
                force=c.frame.reshape(3,3).T@wrench[:3]
                downward+=force[2]*(1 if a==cube_id else -1)
        peak_load=max(peak_load,downward)
        if (i+1)%4:continue
        sensor.step((i+1)*dt,d.sensor('cube_accel').data/(v['gravity_mmps2']*.001))
        rot=d.xmat[cube_id].reshape(3,3)
        top=d.xpos[cube_id,:2]+.022*rot[:2,2]
        pitch=np.rad2deg(np.arcsin(np.clip(d.xmat[m.jnt_bodyid[m.joint('root').id]].reshape(3,3)[2,0],-1,1)))
        trace.append([t,pitch,np.linalg.norm(top-center)*1000,np.rad2deg(np.arctan2(rot[1,0],rot[0,0])-angle),
                      mjcf.lift_height_of_angle(v,float(d.joint("lift_joint").qpos[0]))])
    packets=np.array([[start+t,np.linalg.norm(a)/8192] for t,p in sensor.messages for a in np.frombuffer(p[2:],'<i2').reshape(3,3)])
    return np.array(trace),packets,len(sensor.taps.events),peak_load

initial_checks=[]
firmware='--firmware' in sys.argv[2:]
fig,axes=plt.subplots(4 if firmware else 3,1,sharex=True,figsize=(9,9 if firmware else 7),layout='constrained')
if firmware:axes[3].plot(approach['s'],approach['lift_mm'],color='black')
axes[0].plot(approach['s'],np.rad2deg(approach['pitch']),color='black',label='Physical')
if not held_out:
    opt=np.genfromtxt(here/'optical-travel.csv',delimiter=',',names=True)
    ok=opt['correlation']>.7
    axes[1].scatter(opt['s_after_command'][ok],opt['travel_mm'][ok],color='black',s=9)
force=np.genfromtxt(here/'cube.csv.gz',delimiter=',',names=True)
axes[2].scatter(force['sec']-zero,np.linalg.norm([force[k] for k in 'xyz'],axis=0)/8192,color='black',s=4)
stem='firmware-drive' if firmware else 'free-chassis'
with (here/(stem+'.csv')).open('w') as f:
    writer=csv.writer(f,lineterminator='\n')
    writer.writerow((['onset_s'] if firmware else [])+['release_s','limit_mNm','peak_pitch_deg','settled_travel_mm','settled_yaw_deg','peak_g','taps','peak_downward_fork_N'])
    cases=([(start+shift,30,onset) for start in (3.5,3.8) for onset in (early_onset,late_onset)] if firmware
           else [(3.5,30,None),(3.8,30,None),(3.8,10,None),(3.8,80,None)])
    for start,limit,onset in cases:
        tr,pk,taps,load=run(start,limit,onset)
        writer.writerow(([onset] if firmware else [])+[start,limit,tr[:,1].max(),*np.median(tr[tr[:,0]>5.5+shift,2:4],axis=0),pk[:,1].max(),taps,load])
        if limit==30:
            for ax,j in zip(axes[:2],[1,2]):ax.plot(tr[:,0],tr[:,j],label=(f'{start:.1f} s release, {onset:.3f} s command' if firmware else f'Free from {start} s'))
            axes[2].plot(pk[:,0],pk[:,1])
            if firmware:axes[3].plot(tr[:,0],tr[:,4])
for ax,label in zip(axes,['Chassis pitch (°)','Top glyph travel (mm)','Cube specific force (g)','Lift height (mm)']):
    ax.set(xlim=(3.45+shift,5.75+shift),ylabel=label)
axes[0].legend();axes[-1].set_xlabel('Seconds after wheelstand command')
fig.savefig(here/(stem+'.png'),dpi=155)

if held_out:
    with (here/'initial-contact-check.csv').open('w') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['release_s','onset_s','initial_robot_cube_overlap_mm','cube_settling_displacement_mm'])
        writer.writerows(initial_checks)
