"""Test the viewer surfaces against the compound cube along a frozen trajectory."""
import csv
import json
import subprocess
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from yaw_transfer import cube_rotation, here, load_rows, run


def intersects(tris, center, half):
    local=tris-center
    # The three box axes give a cheap exact rejection before the remaining ten axes.
    keep=((local.min(axis=1)<=half)&(local.max(axis=1)>=-half)).all(axis=1)
    indices=np.flatnonzero(keep)
    if not len(indices):return indices
    tri=local[indices]
    edges=np.roll(tri,-1,axis=1)-tri
    normals=np.cross(edges[:,0],edges[:,1])[:,None,:]
    axes=np.concatenate([normals,np.cross(edges[:,:,None,:],np.eye(3)).reshape(-1,9,3)],axis=1)
    projections=np.einsum('nvc,nac->nva',tri,axes)
    radius=abs(axes)@half
    inside=((projections.min(axis=1)<=radius+1e-14)&(projections.max(axis=1)>=-radius-1e-14)).all(axis=1)
    return indices[inside]


def replay(name,fixture,parts):
    directory=here.parent/name
    view='Before roll' if name=='R8' else 'Next approach'
    yaw=next(float(r['grid_yaw_minus_sdk_deg']) for r in load_rows('optical-registration.csv',here)
             if r['calibration']=='Dock and first cube' and r['view']==view)
    rows=[]
    touched={k:set() for k in parts}
    def observe(m,d):
        if round(d.time/.00025)%80 or not 3<=d.time<=5.208:return
        robot=Rotation.from_quat(d.mocap_quat[0],scalar_first=True)
        pose=d.joint('cube_root').qpos
        inv=Rotation.from_quat(pose[3:],scalar_first=True).inv()
        translation=inv.apply(d.mocap_pos[0]-pose[:3])
        boxes=np.flatnonzero(m.geom_bodyid==m.body('cube').id)
        for part,coords in parts.items():
            tri=(inv*robot).apply(coords).reshape(-1,3,3)+translation
            # Crop to cube envelope before testing individual axis-aligned boxes.
            possible=((tri.min(axis=1)<=.022151)&(tri.max(axis=1)>=-.022151)).all(axis=1)
            tri=tri[possible]
            hits=np.zeros(len(tri),dtype=bool)
            for g in boxes:
                if len(tri):hits[intersects(tri,m.geom_pos[g],m.geom_size[g])]=True
            touched[part].update(np.flatnonzero(possible)[hits])
            rows.append((name,round(d.time,4),part,hits.sum()))
    run(fixture,.00025,True,directory,cube_rotation(directory,yaw),
        {'cube_chip_yaw_deg':180,'cube_accel_scale':[.970709]*3},observer=observe,articulated=True,shelf_support=True)
    return rows,touched


def draw_regions(parts,touched):
    from matplotlib.collections import PolyCollection
    fig,axes=plt.subplots(1,2,figsize=(9,4),layout='constrained')
    for ax,j,ordinate in zip(axes,[1,2],['Left (mm)','Height (mm)']):
        for name in ('body_geo','front apron'):
            triangles=parts[name].reshape(-1,3,3)[:,:,[0,j]]*1000
            ax.add_collection(PolyCollection(triangles,facecolors='none',edgecolors='.75',linewidths=.22))
            marked=triangles[sorted(touched[name])]
            ax.add_collection(PolyCollection(marked,facecolors='tab:orange' if name=='front apron' else 'tab:blue',
                                             alpha=.48,linewidths=0,label=name))
        ax.autoscale()
        ax.set(aspect='equal',xlabel='Forward of front axle (mm)',ylabel=ordinate)
        ax.legend(fontsize=8)
    fig.savefig(here/'shell-regions.png',dpi=150)


if __name__=='__main__':
    parts={k:np.asarray(v) for k,v in json.loads(subprocess.check_output(['node',here/'shell_triangles.mjs'])).items()}
    fig,axes=plt.subplots(2,1,figsize=(8,5.5),layout='constrained',sharex=True)
    with (here/'shell-overlap.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('run','seconds','part','intersecting_triangles'))
        for name,ax in zip(('R8','R9'),axes):
            rows,touched=replay(name,sys.argv[1],parts)
            if name=='R8':
                draw_regions(parts,touched)
            w.writerows(rows)
            for k in parts:
                data=[r for r in rows if r[2]==k]
                ax.plot([r[1] for r in data],[r[3] for r in data],label=k)
                hits=[r[1] for r in data if r[3]]
                print(name,k,(min(hits),max(hits),len(hits)) if hits else 'clear')
            ax.set(title=name,ylabel='Intersecting triangles')
    axes[0].legend(fontsize=8)
    axes[1].set_xlabel('Seconds after roll command')
    fig.savefig(here/'shell-overlap.png',dpi=150)
