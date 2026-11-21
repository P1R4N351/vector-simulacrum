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
zero=22.330433
target=json.loads((here/'initial-cube.json').read_text())
w,x,y,z=[target['q'+str(i)] for i in range(4)]
angle=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
cube_id=m.body('cube').id
cube=m.joint('cube_root').qposadr[0]
def obs(t,k):
    return np.interp(t,approach['s'],approach[k]) if k in approach.dtype.names else np.interp(t+zero,state['sec'],state[k])
def run(start,limit):
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
    for _ in range(round(.20/dt)):mj.mj_step(m,d)
    center=d.qpos[cube:cube+2].copy()
    speed=np.mean([obs(start,'left'),obs(start,'right')])
    d.qvel[:2]=.001*speed*np.array([np.cos(yaw),np.sin(yaw)])
    for axle,radius in [('f',v['wheel_front_radius_mm']),('r',v['wheel_rear_radius_mm'])]:
        for side,key in [('l','left'),('r','right')]:
            d.joint('wheel_'+axle+side+'_joint').qvel[0]=obs(start,key)/radius
    controllers=[TreadController(v),TreadController(v)]
    sensor=CubeAccelerometer(v,sensor_dt,seed=17)
    trace=[];peak_load=0
    for i,t in enumerate(np.arange(start,5.8,dt)):
        if i%round(.001*v['control_period_ms']/dt)==0:
            for controller,side in zip(controllers,['left','right']):
                measured=d.joint('wheel_f'+side[0]+'_joint').qvel[0]*v['wheel_front_radius_mm']
                duty=controller.update(obs(t,side) if t<4.84 else 0,measured)
                d.actuator(side+'_wheel').ctrl[0]=duty*3.92/v['motor_reference_voltage_v']
            d.actuator('lift').ctrl[0]=mjcf.lift_angle_of_height(v,obs(t,'lift_mm'))
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
        trace.append([t,pitch,np.linalg.norm(top-center)*1000,np.rad2deg(np.arctan2(rot[1,0],rot[0,0])-angle)])
    packets=np.array([[start+t,np.linalg.norm(a)/8192] for t,p in sensor.messages for a in np.frombuffer(p[2:],'<i2').reshape(3,3)])
    return np.array(trace),packets,len(sensor.taps.events),peak_load

fig,axes=plt.subplots(3,1,sharex=True,figsize=(9,7),layout='constrained')
axes[0].plot(approach['s'],np.rad2deg(approach['pitch']),color='black',label='Physical')
opt=np.genfromtxt(here/'optical-travel.csv',delimiter=',',names=True)
ok=opt['correlation']>.7
axes[1].scatter(opt['s_after_command'][ok],opt['travel_mm'][ok],color='black',s=9)
force=np.genfromtxt(here/'cube.csv.gz',delimiter=',',names=True)
axes[2].scatter(force['sec']-zero,np.linalg.norm([force[k] for k in 'xyz'],axis=0)/8192,color='black',s=4)
with (here/'free-chassis.csv').open('w') as f:
    writer=csv.writer(f,lineterminator='\n')
    writer.writerow(['release_s','limit_mNm','peak_pitch_deg','settled_travel_mm','settled_yaw_deg','peak_g','taps','peak_downward_fork_N'])
    for start,limit in [(3.5,30),(3.8,30),(3.8,10),(3.8,80)]:
        tr,pk,taps,load=run(start,limit)
        writer.writerow([start,limit,tr[:,1].max(),*np.median(tr[tr[:,0]>5.5,2:],axis=0),pk[:,1].max(),taps,load])
        if limit==30:
            for ax,j in zip(axes[:2],[1,2]):ax.plot(tr[:,0],tr[:,j],label=f'Free from {start} s')
            axes[2].plot(pk[:,0],pk[:,1])
for ax,label in zip(axes,['Chassis pitch (°)','Top glyph travel (mm)','Cube specific force (g)']):
    ax.set(xlim=(3.45,5.75),ylabel=label)
axes[0].legend();axes[-1].set_xlabel('Seconds after wheelstand command')
fig.savefig(here/'free-chassis.png',dpi=155)
