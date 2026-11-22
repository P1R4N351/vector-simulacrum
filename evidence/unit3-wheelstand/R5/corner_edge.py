"""Test the lateral ray depth at loaded fork/cube contacts against 3-D distances."""
import csv
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import read
from triangle_clearance import nearest_distance, outer_face

here = Path(__file__).resolve().parent
sys.path[:0] = [str(here.parents[2]/"experiments/righting"),str(here.parents[1]/"unit3-cube-accelerometer")]
from light_cube import sdk_surface, inside
from fork_envelope import registered_fork

_,cube = sdk_surface(Path(sys.argv[1]))
cube *= 1000
fork = registered_fork(Path(sys.argv[2]))
z_edge = np.unique(cube[:,:,2])
rows=[]
sweep=[]
fig,axes=plt.subplots(1,2,figsize=(8,4),sharey=True,layout="constrained")
for step,folder,ax in zip((.25,.125),sys.argv[3:],axes,strict=True):
    f=read(Path(folder)/"approach-yaw-contacts.csv")
    f=f[f["sec"]<2.159]
    assert set(f["cube_geom"])=={"cube_shell_12"}
    def points(prefix):
        return 1000*np.column_stack([f[f"{prefix}_{dim}_m"] for dim in "xyz"])
    p=points("cube")
    axis=np.argmax(abs(p[:,:2]),axis=1)
    ray=np.empty(len(p))
    for k in (0,1):
        sel=axis==k
        ray[sel]=22-outer_face(p[sel],k,cube)
    # Freeze the fork at the earliest deep-ray contact. Move along +x into
    # the apparent opening on the -x face, holding all other pose components.
    idx=np.flatnonzero((ray>1)&(axis==0))[0]
    poses=np.loadtxt(Path(folder)/"relative-tool.csv",delimiter=",",skiprows=1)
    at=poses[np.argmin(abs(poses[:,0]-f["sec"][idx]))]
    assert abs(at[0]-f["sec"][idx])<1e-9
    transformed=fork@at[4:].reshape(3,3).T + at[1:4]*1000
    # Limit to a 5 mm neighbourhood of the contact; an intersection witness
    # anywhere in this patch is sufficient to rule out free insertion.
    local=transformed[((transformed.min(axis=1)<=p[idx]+5)&
                       (transformed.max(axis=1)>=p[idx]-5)).all(axis=1)]
    grid=np.array([(i/20,j/20) for i in range(21) for j in range(21-i)])
    sample=(local[:,None,0]+(local[:,1]-local[:,0])[:,None]*grid[:,0,None]
            +(local[:,2]-local[:,0])[:,None]*grid[:,1,None]).reshape(-1,3)
    sample=np.unique(sample[np.linalg.norm(sample-p[idx],axis=1)<=5].round(7),axis=0)
    outside=~inside(sample,cube)
    for offset in (0,.1,.5,1.):
        moved=sample[outside]+np.array([offset,0,0])
        crossed=moved[inside(moved,cube)]
        depth=nearest_distance(crossed,cube).max() if len(crossed) else 0
        sweep.append([step,at[0],offset,len(moved),len(crossed),depth])
    dc=nearest_distance(p,cube)
    df=nearest_distance(points("fork"),fork)
    edge=np.min(abs(p[:,2,None]-z_edge),axis=1)
    for name,mask in (("all",np.ones(len(p),bool)),("deep ray",ray>1)):
        w=f["normal_N"][mask]
        rows.append([step,name,w.sum()/sum(f["normal_N"]),ray[mask].max(),dc[mask].max(),
                     df[mask].max(),edge[mask].max(),np.average(dc[mask]+df[mask],weights=w)])
    ax.scatter(ray,dc,s=6,label="Cube OBJ",alpha=.4)
    ax.scatter(ray,df,s=6,label="Fork STL",alpha=.4)
    ax.set(xlabel="Lateral ray depth (mm)",title=f"{step} ms step")
axes[0].set_ylabel("Nearest surface distance (mm)")
axes[0].legend()
fig.savefig(here/"corner-edge.png",dpi=160)
with (here/"corner-edge.csv").open("w") as stream:
    w=csv.writer(stream,lineterminator="\n")
    w.writerow("step_ms subset impulse_fraction max_ray_mm max_cube_distance_mm max_fork_distance_mm max_z_distance_to_sdk_plane_mm weighted_mean_distance_sum_mm".split())
    w.writerows(rows)

with (here/"corner-sweep.csv").open("w") as stream:
    w=csv.writer(stream,lineterminator="\n")
    w.writerow("step_ms pose_s translation_mm baseline_exterior_samples newly_inside_samples max_witness_depth_mm".split())
    w.writerows(sweep)
