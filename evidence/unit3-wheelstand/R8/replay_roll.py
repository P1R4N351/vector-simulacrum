"""Drive the printed fork with recorded SDK poses. Pass the generated printed-fork fixture XML."""
import csv
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

here = Path(__file__).resolve().parent
with (here/'roll-poses.csv').open() as f:
    rows = list(csv.DictReader(f))
states = [r for r in rows if r['source'] == 'robot_state']
cube = next(r for r in rows if r['source'] == 'robot_observed_object' and float(r['sec']) < 0)
assert all(r['origin'] == cube['origin'] for r in states)
with_support = '--front-right' in sys.argv
tree = ET.parse(sys.argv[1])
if with_support:
    sys.path.insert(0,str(here.parents[2]))
    from simulacrum import mjcf, params
    robot = ET.fromstring(mjcf.build_mjcf(params.values(params.load(model='vector_2'))))
    support = robot.find(".//geom[@name='tip_front_r']")
    support.attrib.update(contype="1", conaffinity="6")
    body = ET.SubElement(tree.find('worldbody'),'body',name='driven_chassis',mocap='true')
    body.append(support)
model = mj.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding='unicode'))
data = mj.MjData(model)
fixture = data.joint('cube_root').qpos
cyaw = float(cube['yaw_rad'])
fixture[:] = [float(cube['x_mm'])/1000, float(cube['y_mm'])/1000, .022,
              np.cos(cyaw/2), 0, 0, np.sin(cyaw/2)]
src = np.array([[float(r[k]) for k in ('sec','x_mm','y_mm','yaw_rad','pitch_rad','lift_mm')] for r in states])
src = src[np.argsort(src[:,0])]
src[:,3] = np.unwrap(src[:,3])
fork_id, cube_id = model.body('fork').id, model.body('cube').id
trace = []
for t in np.arange(0, src[-1,0], model.opt.timestep):
    x,y,yaw,pitch,lift = [np.interp(t,src[:,0],col) for col in src[:,1:].T]
    if "--level" in sys.argv:
        pitch = 0.0
    wrist = -41 + np.sqrt(66**2 - (lift-45)**2)
    # Pitch about the chassis origin at rear axle height; positive SDK pitch raises the nose.
    z0 = 16
    reach = wrist*np.cos(pitch) - (lift-z0)*np.sin(pitch)
    z = z0 + wrist*np.sin(pitch) + (lift-z0)*np.cos(pitch)
    data.mocap_pos[0] = np.array([x+reach*np.cos(yaw),y+reach*np.sin(yaw),z])/1000
    c,s = np.cos(yaw/2),np.sin(yaw/2)
    cp,sp = np.cos(pitch/2),-np.sin(pitch/2)
    data.mocap_quat[0] = [c*cp,-s*sp,c*sp,s*cp]
    if with_support:
        i = model.body('driven_chassis').mocapid[0]
        data.mocap_pos[i] = np.array([x+z0*np.sin(pitch)*np.cos(yaw),
                                     y+z0*np.sin(pitch)*np.sin(yaw),z0*(1-np.cos(pitch))])/1000
        data.mocap_quat[i] = data.mocap_quat[0]
    mj.mj_step(model,data)
    contact = any(set(model.geom_bodyid[[ct.geom1,ct.geom2]]) == {cube_id,fork_id}
                  for ct in data.contact)
    tilt = np.degrees(np.arccos(np.clip(data.body('cube').xmat[-1],-1,1)))
    front_contact = with_support and any(set(model.geom_bodyid[[ct.geom1,ct.geom2]]) ==
                                             {cube_id,model.body('driven_chassis').id} for ct in data.contact)
    trace.append((t,contact,tilt,front_contact))
t,contact,tilt,front_contact = np.array(trace).T
if np.any(front_contact):
    print('first support contact',t[np.flatnonzero(front_contact)[0]])
change = np.diff(contact, prepend=0) > 0
print('first contacts',t[change][:6], 'peak tilt',max(tilt), 'last tilt',tilt[-1])
# Horizontal projection of the original top-face normal versus cube-to-robot bearing.
normal = data.body('cube').xmat.reshape(3,3)[:2,2]
bearing = src[-1,1:3]/1000 - fixture[:2]
direction = np.degrees(np.arccos(np.clip(normal@bearing/(np.linalg.norm(normal)*np.linalg.norm(bearing)),-1,1)))
print('first 45 degrees', t[np.flatnonzero(tilt >= 45)[0]] if max(tilt)>=45 else None,
      'angle from robot bearing', direction)
acc = np.genfromtxt(here/'cube.csv.gz',delimiter=',',names=True)
zero = next(float(r['sec']) for r in csv.DictReader((here/'actions.csv').read_text().splitlines()) if r['action']=='roll cube')
fig, ax = plt.subplots(2,1,figsize=(8,4.8),sharex=True,layout='constrained')
ax[0].plot(acc['sec']-zero,np.linalg.norm(np.column_stack([acc[k] for k in 'xyz']),axis=1)/8192,'.',ms=2)
ax[0].set(ylabel='Measured cube (g)',title='Native roll on the shelf')
ax[1].plot(t,tilt,label='Simulated cube tilt')
ax[1].fill_between(t,0,1,where=contact.astype(bool),transform=ax[1].get_xaxis_transform(),alpha=.15,label='Fork contact')
if with_support:
    ax[1].fill_between(t,0,.1,where=front_contact.astype(bool),transform=ax[1].get_xaxis_transform(),
                       color='tab:orange',alpha=.7,label='Front-right tread proxy contact')
ax[1].legend()
ax[1].set(xlabel='Seconds after roll command',ylabel='Tilt (degrees)',xlim=(0,5.3))
fig.savefig(here/('front-right-replay.png' if with_support else 'contact-level.png' if '--level' in sys.argv else 'contact-replay.png'),dpi=150)
