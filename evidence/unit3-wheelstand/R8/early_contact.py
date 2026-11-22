"""Reconstruct part-to-cube distances across the first roll tap (mm, roll-relative s)."""
import csv
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

here = Path(__file__).resolve().parent
repo = here.parents[2]
sys.path[:0] = [str(repo), str(repo/'evidence/unit3-printed-fork-contact')]
from replay_links import add_links, rail_paths
from simulacrum import mjcf, params

def rows(name):
    with (here/name).open() as f:
        return list(csv.DictReader(f))

v = params.values(params.load(model='vector_2'))
tree = ET.ElementTree(ET.fromstring(mjcf.build_mjcf(v, cube='free')))
add_links(tree,repo)
m = mj.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding='unicode'))
d = mj.MjData(m)
cube_geoms = np.flatnonzero(m.geom_bodyid==m.body('cube').id)
groups = {'chassis':['chassis_geom'],'head':['head_geom'],
          'wheels':[m.geom(i).name for i in range(m.ngeom) if m.geom(i).name.startswith('wheel_')],
          'tip supports':[m.geom(i).name for i in range(m.ngeom)
                                    if m.geom(i).name.startswith(('tip_front','tip_rear'))]}
for j,part in enumerate(('upper links','lower links')):
    groups[part] = [m.geom(i).name for i in np.flatnonzero(m.geom_bodyid==m.body(f'rail{j}').id)]
assert all(groups.values())
poses = rows('roll-poses.csv')
r = [row for row in poses if row['source']=='robot_state']
targets = [row for row in poses if row['source']=='robot_observed_object']
assert len({row['origin'] for row in poses}) == 1
src = np.array([[float(row[k]) for k in ('sec','x_mm','y_mm','yaw_rad','pitch_rad','lift_mm')] for row in r])
src = src[np.argsort(src[:,0])]
src[:,3] = np.unwrap(src[:,3])
stream = np.genfromtxt(here/'state.csv.gz',names=True,delimiter=',')
zero = next(float(r['sec']) for r in rows('actions.csv') if r['action']=='roll cube')
taps = next(row['seconds_after_roll'] for row in rows('event-replay.csv') if row['series']=='Counter change')
first = float(taps.split()[0])
times = np.unique(np.r_[np.arange(2.7,3.501,.02),first])
x,y,yaw,pitch,lift = [np.interp(times,src[:,0],col) for col in src[:,1:].T]
centers, link_q = rail_paths(repo, np.zeros_like(x),np.zeros_like(y),np.zeros_like(yaw),lift)
rail_ids = [m.body(f'rail{i}').mocapid[0] for i in range(2)]
series = {name:np.zeros((len(targets),len(times))) for name in groups}
with (here/'early-contact.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('cube_observation_s','roll_s','robot_part','signed_gap_mm','nearest_robot_geom','nearest_cube_geom'))
    for j,ob in enumerate(targets):
        a=float(ob['yaw_rad'])/2
        d.joint('cube_root').qpos[:]=[float(ob['x_mm'])/1000,float(ob['y_mm'])/1000,.022,np.cos(a),0,0,np.sin(a)]
        for i,t in enumerate(times):
            rot=Rotation.from_euler('ZY',[yaw[i],-pitch[i]])
            pivot=np.array([0,0,.016])
            origin=np.array([x[i]/1000,y[i]/1000,0])+pivot-rot.apply(pivot)
            d.joint('root').qpos[:]=np.r_[origin,rot.as_quat(scalar_first=True)]
            angle=mjcf.lift_angle_of_height(v,lift[i])
            d.joint('lift_joint').qpos[0]=angle
            d.joint('wrist_joint').qpos[0]=-angle
            d.joint('head_joint').qpos[0]=np.interp(t+zero,stream['sec'],stream['head'])
            d.mocap_pos[rail_ids]=origin+rot.apply(centers[i])
            d.mocap_quat[rail_ids]=(rot*Rotation.from_quat(link_q[i],scalar_first=True)).as_quat(scalar_first=True)
            mj.mj_forward(m,d)
            for part,names in groups.items():
                gap,near,cg=min((1000*mj.mj_geomDistance(m,d,m.geom(name).id,g,.25,None),name,m.geom(g).name)
                                for name in names for g in cube_geoms)
                series[part][j,i]=gap
                w.writerow((ob['sec'],round(t,6),part,round(gap,4),near,cg))
fig,ax=plt.subplots(figsize=(8,4.3),layout='constrained')
for part,gaps in series.items():
    line,=ax.plot(times,gaps[0],label=part)
    ax.fill_between(times,gaps.min(axis=0),gaps.max(axis=0),color=line.get_color(),alpha=.2)
    print(part,np.round(gaps[:,np.searchsorted(times,first)],1),'range minimum',round(gaps.min(),1))
ax.axvline(first,c='.4',ls=':',label='First cube counter change')
ax.axhline(0,c='.4',lw=.8)
ax.set(xlabel='Seconds after roll command',ylabel='Closest signed gap (mm)',title='Clearance to the cube during the first tap')
ax.legend(fontsize=8,ncol=2)
fig.savefig(here/'early-contact.png',dpi=160)
