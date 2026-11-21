"""Free shelf cube driven only by the fork and links following the recorded approach."""
from pathlib import Path
from xml.etree import ElementTree as ET
import csv
import sys

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

here=Path(__file__).resolve().parent
repo=here.parents[1]
sys.path[:0]=[str(repo),str(here.parent/"unit3-printed-fork-contact")]
from replay_links import add_links,rail_paths
from simulacrum import params
import pickup_patch
from decode import decode
from pickup_support import contact_load, save as save_support


def main():
    tree=ET.parse(sys.argv[1])
    assert len(tree.findall(".//body[@name='fork']/geom"))==58
    add_links(tree,repo)
    m=mj.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding="unicode"))
    path=np.genfromtxt(here/"pickup-path.csv",names=True,delimiter=",")
    sights=np.genfromtxt(here/"pickup-sightings.csv",names=True,delimiter=",")
    assert np.all(path['origin_id']==sights[0]['origin_id'])
    v=params.values(params.load(model="vector_2"))
    t=np.arange(-.3,13.1,m.opt.timestep)
    pose_clock=path['time_s'].copy()
    smooth_receipt='--smooth-receipt' in sys.argv[2:]
    if smooth_receipt:
        pose_clock[1:-1]=(pose_clock[:-2]+2*pose_clock[1:-1]+pose_clock[2:])/4
    x,y,yaw,lift=[np.interp(t,pose_clock,path[k]) for k in ('x_mm','y_mm','yaw_rad','lift_mm')]
    bx,_,bz=v['lift_base_position_mm']
    reach=bx+np.sqrt(v['lift_arm_length_mm']**2-(lift-bz)**2)
    wrist=.001*np.column_stack([x+reach*np.cos(yaw),y+reach*np.sin(yaw),lift])
    wristq=np.column_stack([np.cos(yaw/2),np.zeros_like(t),np.zeros_like(t),np.sin(yaw/2)])
    rails,railq=rail_paths(repo,x,y,yaw,lift)
    f=m.body('fork').mocapid[0]
    ids=[m.body(f'rail{i}').mocapid[0] for i in range(2)]
    patch=[]
    support=[]
    headings=[]
    output=[]
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
    for label,s in [('Before command',sights[sights['time_s']<0][-1]),('Last sighting',sights[-1]),('Last sighting + pitch',sights[-1])]:
        paths=wrist
        orientations=wristq
        pivots=rails
        link_rot=railq
        if label.endswith("+ pitch"):
            # SDK positive pitch raises the nose, about the transverse drive axle.
            pitch=np.interp(t,pose_clock,path['pitch_rad'])
            angle=Rotation.from_rotvec(np.column_stack([np.sin(yaw)*pitch,-np.cos(yaw)*pitch,np.zeros_like(t)]))
            axis=.001*np.column_stack([x,y,np.full_like(t,v["wheel_front_radius_mm"])])
            paths=axis+angle.apply(wrist-axis)
            pivots=np.stack([axis+angle.apply(rails[:,j]-axis) for j in range(2)],axis=1)
            orientations=(angle*Rotation.from_quat(wristq,scalar_first=True)).as_quat(scalar_first=True)
            link_rot=(angle*Rotation.from_quat(railq,scalar_first=True)).as_quat(scalar_first=True)
        d=mj.MjData(m)
        pose=d.joint('cube_root').qpos
        # Use the camera estimate in the shelf plane. Gravity settles the cube on its rubber pads.
        h=Rotation.from_quat([s['qx'],s['qy'],s['qz'],s['qw']]).as_euler('xyz')[2]
        pose[:]=[s['x_mm']/1000,s['y_mm']/1000,.0222,np.cos(h/2),0,0,np.sin(h/2)]
        sample=[]
        since_contact=None
        floor=m.geom('table').id
        cubegeoms={g for g in range(m.ngeom) if m.geom_bodyid[g]==m.body('cube').id}
        window=[]
        cube=m.body('cube').id
        weight=m.body_mass[cube]*np.linalg.norm(m.opt.gravity)
        for i,clock in enumerate(t):
            d.mocap_pos[f]=paths[i]
            d.mocap_quat[f]=orientations[i]
            d.mocap_pos[ids]=pivots[i]
            d.mocap_quat[ids]=link_rot[i]
            mj.mj_step(m,d)
            if 1.99 <= clock <= 3.61:
                if label.endswith("+ pitch") and not smooth_receipt:
                    pickup_patch.collect(m,d,cube,m.body("fork").id,clock,m.opt.timestep,patch)
                window.append(contact_load(m,d,cube,floor,m.body('fork').id)/weight)
                if len(window)==10:
                    support.append([label,round(clock,4),round(lift[i],3),*np.mean(window,axis=0).round(5)])
                    window.clear()
            pairs=[c.geom for c in d.contact if any(g in cubegeoms for g in c.geom)]
            floor_contact=any(floor in pair for pair in pairs)
            fork_contact=any(floor not in pair for pair in pairs)
            if fork_contact and since_contact is None:
                since_contact=clock
            if i%10==0:
                tilt=np.degrees(np.arccos(np.clip(1-2*(pose[4]**2+pose[5]**2),-1,1)))
                if 1.0 <= clock <= 3.0:
                    qw,qx,qy,qz=pose[3:7]
                    heading=np.arctan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
                    delta=heading-h
                    headings.append([label,round(clock,4),round(np.degrees(np.arctan2(np.sin(delta),np.cos(delta))),3)])
                sample.append([clock,pose[2]*1000,tilt,int(floor_contact),int(fork_contact),pose[0]*1000-s["x_mm"],pose[1]*1000-s["y_mm"]])
        a=np.array(sample)
        for ax,j in zip(axes,[1,2]):
            ax.plot(a[:,0],a[:,j],label=label)
        output.extend([[label,*row] for row in sample])
        high=(a[:,0]>6.5)&(a[:,0]<8.3)
        print(label,'first contact',since_contact,'max center height',a[:,1].max(),'held pose',np.median(a[high,1:3],axis=0),'floor fraction',a[high,3].mean())
    if smooth_receipt:
        save_support(here,support,stem='pickup-support-smoothed')
        return
    pickup_patch.save(here,patch)
    save_support(here,support)
    with (here/'shelf-yaw-sim.csv').open('w') as fobj:
        w=csv.writer(fobj,lineterminator='\n')
        w.writerow(['initialization','time_s','yaw_change_deg'])
        w.writerows(headings)
    pkt,_=decode(here/'pickup.tsv')
    with (here/'action-times.csv').open() as fobj:
        zero=float(next(e["pod_unix_s"] for e in csv.DictReader(fobj) if e["action"]=="pickup"))
    clock=np.array([p[0]*1e-9-zero for p in pkt])
    g=np.array([np.frombuffer(p[3],"<i2").reshape(3,3).mean(0) for p in pkt])
    mask=(clock>=-.3)&(clock<=13.1)
    axes[1].plot(clock[mask],np.degrees(np.arctan2(np.linalg.norm(g[mask,:2],axis=1),g[mask,2])),"k.",ms=2,label="Cube apparent tilt")
    axes[0].set_ylabel("Simulated center height (mm)")
    axes[1].set(ylabel="Tilt (°)",xlabel="Seconds from pickup command")
    for ax in axes:
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(here/'free-pickup.png',dpi=150)
    with (here/'free-pickup.csv').open('w') as fobj:
        w=csv.writer(fobj,lineterminator='\n')
        w.writerow(['initialization','time_s','center_z_mm','tilt_deg','touches_floor','touches_fork_or_links','cube_dx_mm','cube_dy_mm'])
        w.writerows(output)

if __name__=='__main__':
    main()
