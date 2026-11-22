"""Check the hard corner against the normalized SDK surface and the R5 push."""
import csv
import hashlib
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np
from pickup_onset import read

here = Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[2]/"experiments/righting"))
from light_cube import sdk_surface

obj = Path(sys.argv[1])
_,tri = sdk_surface(obj)
tri *= 1000
def outer_face(points,axis):
    """Outermost triangle intersection on the lateral axis at each contact."""
    uv=[i for i in range(3) if i!=axis]
    extent=np.full(len(points),-np.inf)
    for a,b,c in tri:
        matrix=np.column_stack(((b-a)[uv],(c-a)[uv]))
        if abs(np.linalg.det(matrix)) < 1e-10:
            continue
        weights=(points[:,uv]-a[uv])@np.linalg.inv(matrix).T
        mask=(weights.min(axis=1)>=-1e-9)&(weights.sum(axis=1)<=1+1e-9)
        coord=a[axis]+weights[mask]@np.array([b[axis]-a[axis],c[axis]-a[axis]])
        extent[mask]=np.maximum(extent[mask],coord*(1 if axis else -1))
    return extent

fig,axes=plt.subplots(2,1,sharex=True,figsize=(8,6),layout="constrained")
g=read(here/"approach-glyph-axes.csv")
g=g[(g["width_mm"]==23)&(g["sec"]>1.95)]
rows=[]
surface=[]
for j,path in enumerate(map(Path,sys.argv[2:])):
    step=(.25,.125)[j%2]
    trim=(0,.5,1)[j//2]
    h=read(path/"cube-heading.csv")
    pose=read(path/"pickup-replay.csv")
    yaw=np.rad2deg(np.unwrap(h["yaw_rad"])-h["yaw_rad"][0])
    mask=(h["sec"]>1.95)&(h["sec"]<2.35)
    axes[j%2].plot(h["sec"][mask],yaw[mask],label=f"{trim:g} mm inset")
    f=read(path/"approach-yaw-contacts.csv")
    ht=np.interp([2.334498,3.254498],pose["sec"],pose["center_height_mm"])
    rows.append([step,trim,f["cube_geom"][0],f["sec"][0],np.interp(2.334498,h["sec"],yaw),ht[1]-ht[0]])
    if trim==0:
        f=f[f["sec"]<2.159]
        assert set(f["cube_geom"])=={"cube_shell_12"}
        xyz=np.column_stack([f[name] for name in ("cube_x_m","cube_y_m","cube_z_m")])*1000
        axis=np.argmax(abs(xyz[:,:2]),axis=1)
        for k in (0,1):
            selected=axis==k
            exterior=outer_face(xyz[selected],k)
            weights=f["normal_N"][selected]
            valid=np.isfinite(exterior)
            clearance=22-exterior[valid]
            order=np.argsort(clearance)
            cumulative=np.cumsum(weights[valid][order]); cumulative/=cumulative[-1]
            surface.append([step,"-x" if k==0 else "+y",weights.sum()/f["normal_N"].sum(),
                            weights[valid].sum()/weights.sum(),weights[valid][abs(clearance)<=.05].sum()/weights.sum(),*np.interp([.05,.5,.95],cumulative,clearance[order]),
                            hashlib.sha256(obj.read_bytes()).hexdigest()])
for ax,step in zip(axes,(.25,.125)):
    ax.plot(g["sec"],g["yaw_change_deg"],"ko-",ms=3,label="Shelf glyph")
    ax.set_ylabel(f"Yaw change (°)\n{step} ms step")
    ax.legend(fontsize=7,ncol=4)
axes[-1].set_xlabel("Seconds from pickup command")
fig.savefig(here/"corner-clearance.png",dpi=160)
for name,columns,data in [
    ("corner-clearance","step_ms trim_mm first_piece first_s frame85_yaw_deg frame106_rise_mm",rows),
    ("corner-surface","step_ms face impulse_fraction ray_hit_fraction flush_fraction_0p05mm depth_p05_mm depth_p50_mm depth_p95_mm sdk_sha256",surface)]:
    with (here/(name+".csv")).open("w") as stream:
        w=csv.writer(stream,lineterminator="\n")
        w.writerow(columns.split());w.writerows(data)
