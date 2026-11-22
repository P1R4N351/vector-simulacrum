"""Replay the same driven fork and translating tread with and without tangential tread speed."""
import csv
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[2]))
from simulacrum import mjcf, params
from simulacrum.cube import CubeAccelerometer


def load_rows(filename, directory=here):
    with (directory/filename).open() as f:
        return list(csv.DictReader(f))


def compose_fixture(path, values):
    root = ET.parse(path).getroot()
    robot = ET.fromstring(mjcf.build_mjcf(values,cube='free'))
    root.find(".//body[@name='cube']").append(robot.find(".//site[@name='cube_chip']"))
    ET.SubElement(ET.SubElement(root,'sensor'),'accelerometer',site='cube_chip',name='chip')
    geom = robot.find(".//geom[@name='tip_front_r']")
    ends = np.fromstring(geom.attrib.pop('fromto'), sep=' ').reshape(2,3)
    center = ends.mean(axis=0)
    radius = float(geom.get('size'))
    # Cylinder's native axis is Z. Orient it along the wheel axle (Y).
    geom.attrib.update(size=f'{radius} {np.linalg.norm(ends[1]-ends[0])/2}',
                       quat='.707106781186548 -.707106781186548 0 0',
                       contype='1',conaffinity='6')
    body = ET.SubElement(root.find('worldbody'),'body',name='tread')
    ET.SubElement(body,'freejoint',name='tread_path')
    # The driver resets the pose and velocity each step. An explicit inertia controls
    # how much the proxy yields to the contact solver within that one step.
    ET.SubElement(body,'inertial',pos='0 0 0',mass='.18',diaginertia='.0001 .0001 .0001')
    body.append(geom)
    return root,center,radius


def run(path, dt, rolling, directory=here, initial_rotation=None, sensor_overrides=None, observer=None, articulated=False, initial_xy_mm=None):
    values = params.values(params.load(model="vector_2"))
    values.update(sensor_overrides or {})
    root,center,radius = compose_fixture(path,values)
    if articulated:
        robot=ET.fromstring(mjcf.build_mjcf(values))
        ET.SubElement(root,'compiler',angle='radian')
        original=root.find(".//body[@name='fork']")
        arm=robot.find(".//body[@name='lift_arm']")
        wrist_body=arm.find("body")
        wrist_body.set('name','fork')
        for geom in wrist_body.findall('geom'):
            wrist_body.remove(geom)
        for geom in original.findall('geom'):
            original.remove(geom)
            wrist_body.append(geom)
        ET.SubElement(wrist_body,'inertial',pos='0 0 -.010',mass='.006',diaginertia='2e-6 1e-6 2e-6')
        original.set('name','carrier')
        original.append(arm)
        ET.SubElement(root,'actuator').append(robot.find(".//position[@name='lift']"))
        equality=ET.SubElement(root,'equality')
        equality.append(robot.find(".//equality/joint[@joint1='wrist_joint']"))
    root.find('option').set('timestep',str(dt))
    m = mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    d = mj.MjData(m)
    rows = load_rows('roll-poses.csv',directory)
    cube = next(r for r in rows if r['source']=='robot_observed_object' and float(r['sec'])<0)
    if initial_rotation is None:
        initial_rotation = Rotation.from_euler('z',float(cube['yaw_rad']))
    up = initial_rotation.inv().apply([0,0,1])
    xy = np.array([float(cube['x_mm']),float(cube['y_mm'])] if initial_xy_mm is None else initial_xy_mm)
    d.joint('cube_root').qpos[:] = [*(xy/1000),.022,
                                    *initial_rotation.as_quat(scalar_first=True)]
    states = sorted((r for r in rows if r['source']=='robot_state'),key=lambda r:float(r['sec']))
    samples = np.array([[float(r[k]) for k in ('sec','x_mm','y_mm','yaw_rad','pitch_rad','lift_mm')] for r in states])
    samples[:,3] = np.unwrap(samples[:,3])
    t = np.arange(0,samples[-1,0],dt)
    x,y,yaw,pitch,lift = [np.interp(t,samples[:,0],a) for a in samples[:,1:].T]
    rot = Rotation.from_euler('ZY',np.column_stack((yaw,-pitch)))
    pivot = np.array([0.,0.,.016])
    origin = np.column_stack((x,y,np.zeros_like(x)))/1000 + pivot - rot.apply(pivot)
    wrist = -41+np.sqrt(66**2-(lift-45)**2)
    forkpos = origin + rot.apply(np.column_stack((wrist,np.zeros_like(x),lift))/1000)
    treadpos = origin + rot.apply(center)
    zero = next(float(r['sec']) for r in load_rows('actions.csv',directory) if r['action']=='roll cube')
    hardware = np.genfromtxt(directory/'state.csv.gz',names=True,delimiter=',')
    speed = np.interp(t,hardware['sec']-zero,hardware['right'])/1000
    spin = np.zeros_like(t)
    if rolling:
        spin[1:] = np.cumsum((speed[:-1]+speed[1:])*.5*dt/radius)
    quat = rot.as_quat(scalar_first=True)
    wheelquat = (rot*Rotation.from_euler('y',spin[:,None])).as_quat(scalar_first=True)
    qref = np.column_stack((treadpos,wheelquat))
    joint = d.joint('tread_path')
    dof = m.joint('tread_path').dofadr[0]
    qadr = m.joint('tread_path').qposadr[0]
    vel = np.zeros(m.nv)
    ids = m.geom_bodyid
    cube_id,tread_id = m.body('cube').id,m.body('tread').id
    sensor = CubeAccelerometer(values,dt)
    tilt, hit = [],[]
    if articulated:
        target=np.array([mjcf.lift_angle_of_height(values,h) for h in lift])
        d.joint('lift_joint').qpos[0]=target[0]
        d.joint('wrist_joint').qpos[0]=-target[0]
    for i in range(len(t)-1):
        d.mocap_pos[0],d.mocap_quat[0] = forkpos[i],quat[i]
        if articulated:
            d.mocap_pos[0]=origin[i]
            d.actuator('lift').ctrl[0]=target[i]
        joint.qpos[:] = qref[i]
        next_q = d.qpos.copy()
        next_q[qadr:qadr+7] = qref[i+1]
        mj.mj_differentiatePos(m,vel,dt,d.qpos,next_q)
        joint.qvel[:] = vel[dof:dof+6]
        mj.mj_step(m,d)
        sensor.step(d.time,d.sensor('chip').data/9.81)
        if observer is not None:
            observer(m,d)
        hit.append(any({ids[c.geom1],ids[c.geom2]}=={cube_id,tread_id} for c in d.contact))
        tilt.append(np.degrees(np.arccos(np.clip(d.body('cube').xmat.reshape(3,3)[2]@up,-1,1))))
    return t[:-1],np.array(tilt),np.array(hit),speed[:-1]*1000,sensor


if __name__=='__main__':
    fig,axes=plt.subplots(3,1,figsize=(8,7),sharex=True,layout='constrained')
    with (here/'tread-speed.csv').open('w') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(('dt_ms','uses_wheel_speed','first_contact_s','peak_tilt_deg','last_tilt_deg','packet_counter_changes_after_2_5s','peak_packet_g_after_2_5s','last_filtered_axis'))
        for dt in (.001,.0005,.00025):
            for rolling in (False,True):
                t,tilt,hit,speed,sensor=run(sys.argv[1],dt,rolling)
                packets=[(stamp,np.frombuffer(payload[2:],'<i2').reshape(3,3)/8192)
                         for stamp,payload in sensor.messages if stamp>2.5]
                stamps=np.repeat([stamp for stamp,_ in packets],3)
                norm=np.linalg.norm(np.vstack([v for _,v in packets]),axis=1)
                writer.writerow((dt*1000,rolling,t[hit][0],max(tilt),tilt[-1],
                                 sum(k>2.5 for k in sensor.taps.events),max(norm),sensor.up_axis.axis))
                if dt==.00025:
                    if rolling:
                        axes[2].plot(stamps,norm,'.',ms=2,label='Simulated packets, moving tread')
                    axes[1].plot(t,tilt,label='Recorded tread speed' if rolling else 'Zero tread speed')
    physical=np.genfromtxt(here/'cube.csv.gz',delimiter=',',names=True)
    zero=next(float(r['sec']) for r in load_rows('actions.csv') if r['action']=='roll cube')
    axes[2].plot(physical['sec']-zero,np.linalg.norm(np.column_stack([physical[k] for k in 'xyz']),axis=1)/8192,
                 '.',ms=2,label='Physical packets')
    axes[2].set(ylabel='Cube acceleration (g)',xlabel='Seconds from roll command')
    axes[2].legend()
    axes[0].plot(t,speed)
    axes[0].axvline(3.128174,ls=':',c='.4',label='First cube tap counter change')
    axes[0].legend()
    axes[0].set(ylabel='Right wheel (mm/s)',title='Native roll with measured chassis and fork motion')
    axes[1].set(xlim=(2.5,5.21),xlabel='Seconds from roll command',ylabel='Simulated cube tilt (°)')
    axes[1].legend()
    fig.savefig(here/'tread-speed.png',dpi=155)
