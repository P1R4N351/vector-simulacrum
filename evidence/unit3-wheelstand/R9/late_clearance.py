"""Measure omitted part clearances along the unmodified, shelf-supported replay."""
import csv
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation
from yaw_transfer import cube_rotation, here, load_rows, run
from simulacrum import mjcf, params

repo = here.parents[2]
sys.path.insert(0,str(repo/'evidence/unit3-printed-fork-contact'))
from replay_links import add_links, rail_paths


def compare(name, fixture):
    directory=here.parent/name
    v=params.values(params.load(model='vector_2'))
    tree=ET.ElementTree(ET.fromstring(mjcf.build_mjcf(v,cube='free')))
    add_links(tree,repo)
    world=tree.find('worldbody')
    stock=world.find("body[@name='cube']")
    detailed=ET.parse(fixture).find(".//body[@name='cube']")
    detailed.append(stock.find("site[@name='cube_chip']"))
    world.remove(stock)
    world.append(detailed)
    m=mj.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding='unicode'))
    d=mj.MjData(m)
    cube=np.flatnonzero(m.geom_bodyid==m.body('cube').id)
    groups={'chassis':['chassis_geom'],'head':['head_geom']}
    for j,label in enumerate(('Upper links','Lower links')):
        groups[label]=[m.geom(i).name for i in np.flatnonzero(m.geom_bodyid==m.body(f'rail{j}').id)]
    groups['Other tread supports']=[m.geom(i).name for i in range(m.ngeom)
                                  if m.geom(i).name.startswith('tip_') and m.geom(i).name!='tip_front_r']
    groups['Inset wheels']=[m.geom(i).name for i in range(m.ngeom) if m.geom(i).name.startswith('wheel_')]
    rail_ids=[m.body(f'rail{i}').mocapid[0] for i in range(2)]
    zero=next(float(r['sec']) for r in load_rows('actions.csv',directory) if r['action']=='roll cube')
    state=np.genfromtxt(directory/'state.csv.gz',delimiter=',',names=True)
    view='Before roll' if name=='R8' else 'Next approach'
    delta=next(float(r['grid_yaw_minus_sdk_deg']) for r in load_rows('optical-registration.csv',here)
               if r['calibration']=='Dock and first cube' and r['view']==view)
    trace=[]
    def observe(model,data):
        step=round(data.time/.00025)
        if step%80 or not 4<=data.time<=5.208: return
        origin=data.mocap_pos[0]
        quat=data.mocap_quat[0]
        rot=Rotation.from_quat(quat,scalar_first=True)
        a=data.joint('lift_joint').qpos[0]
        centers,q=rail_paths(repo,*[np.zeros(1) for _ in range(3)],np.array([45+66*np.sin(a)]))
        d.joint('root').qpos[:]=np.r_[origin,quat]
        d.joint('cube_root').qpos[:]=data.joint('cube_root').qpos
        d.joint('lift_joint').qpos[0]=a
        d.joint('wrist_joint').qpos[0]=data.joint('wrist_joint').qpos[0]
        d.joint('head_joint').qpos[0]=np.interp(data.time+zero,state['sec'],state['head'])
        d.mocap_pos[rail_ids]=origin+rot.apply(centers[0])
        d.mocap_quat[rail_ids]=(rot*Rotation.from_quat(q[0],scalar_first=True)).as_quat(scalar_first=True)
        mj.mj_forward(m,d)
        for group,names in groups.items():
            gap,part=min((1000*mj.mj_geomDistance(m,d,m.geom(key).id,g,.30,None),key)
                         for key in names for g in cube)
            trace.append((name,round(data.time,4),group,part,gap))
    run(fixture,.00025,True,directory,cube_rotation(directory,delta),
        {'cube_chip_yaw_deg':180,'cube_accel_scale':[.970709]*3},observer=observe,articulated=True,shelf_support=True)
    return trace


if __name__=='__main__':
    fig,axes=plt.subplots(2,1,figsize=(9,6.5),layout='constrained',sharex=True)
    with (here/'late-clearance.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('run','seconds','group','nearest_robot_geom','gap_mm'))
        for name,ax in zip(('R8','R9'),axes):
            trace=compare(name,sys.argv[1])
            w.writerows(trace)
            for group in dict.fromkeys(r[2] for r in trace):
                data=[r for r in trace if r[2]==group]
                ax.plot([r[1] for r in data],[r[-1] for r in data],label=group)
                print(min(data,key=lambda r:r[-1]))
            ax.axhline(0,color='.35',ls=':')
            ax.set(title=name,ylabel='Signed gap (mm)')
    axes[0].legend(fontsize=8,ncol=3)
    axes[1].set_xlabel('Seconds after roll command')
    fig.savefig(here/'late-clearance.png',dpi=150)
