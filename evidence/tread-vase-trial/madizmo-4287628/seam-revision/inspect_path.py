from pathlib import Path
import csv, math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from shapely.geometry import LineString,box
from shapely.ops import unary_union
from scipy.signal import find_peaks

here=Path(__file__).resolve().parent
def read(path):
    layers={};position=np.zeros(4);k=-1;idle=[];down=0;retractions=0
    for line in path.read_text().splitlines():
        if line.startswith(';LAYER:'):k=int(line[7:]);layers[k]=[]
        cmd=line.partition(';')[0].split()
        if not cmd or cmd[0] not in ('G0','G1','G92'):continue
        new=position.copy()
        for w in cmd[1:]:
            if w[0] in 'XYZE':new['XYZE'.index(w[0])]=float(w[1:])
        if k>=0 and cmd[0]!='G92' and np.any(new!=position):
            down+=int(new[2]<position[2]);retractions+=int(new[3]<position[3])
            if new[3]>position[3]:layers[k].append([position[:3].copy(),new[:3].copy()])
            else:idle.append(k)
        position=new
    return {k:np.array(d)-[90,90,0] for k,d in layers.items()},idle,down,retractions

def crossings(segments,x):
    a,b=segments[:,0],segments[:,1];cross=(a[:,0]>x)!=(b[:,0]>x)
    q=a[cross]+((x-a[cross,0])/(b[cross,0]-a[cross,0]))[:,None]*(b-a)[cross]
    return q[abs(q[:,1])<.55]

def teeth(seg):
    a,b=seg[:,0,:2],seg[:,1,:2];inner=[]
    for theta in np.deg2rad(np.arange(1.05,359,.5)):
        u=np.array([np.cos(theta),np.sin(theta)]);n=np.array([-u[1],u[0]])
        aa,bb=a@n,b@n; mask=(aa>0)!=(bb>0)
        r=(a[mask]+(aa[mask]/(aa-bb)[mask])[:,None]*(b-a)[mask])@u
        if sum(r>0)==2:inner.append(min(r[r>0]))
    return len(find_peaks(-np.array(inner),prominence=.15)[0])

before=read(here.parent/'kp3s-0.02-preview.gcode')[0]
after,idle,down,retractions=read(here/'kp3s-seam-preview.gcode')
assert list(after)==list(range(58)) and idle==[0] and not down and not retractions
assert all(np.allclose(after[k-1][-1,1],after[k][0,0]) for k in range(1,58))
rows=[]
for k,seg in after.items():
    p=crossings(seg,27)
    # Layer zero gets a foundation pass plus a spiral; retain all intersections in G-code order.
    assert len(p) in [2,4]
    for l,r in zip(p[::2],p[1::2]):
        dx=abs(r[1]-l[1]); dz=abs(r[2]-l[2])
        rows.append((k,dx,.4-dx,dz))
        assert dx<.4 and dz<.2
with (here/'seam-per-layer.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['cura_layer','centerline_spacing_mm','nominal_xy_overlap_mm','cap_height_difference_mm']);w.writerows(rows)
assert all(teeth(after[k])==42 for k in range(24,34))
print('layers',len(after),'seam ranges',np.array(rows)[:,1:].min(0),np.array(rows)[:,1:].max(0),'layer29',rows[30], 'nonextruding moves',idle,'highest Z',after[57][:,:,2].max())
fig,axes=plt.subplots(1,2,figsize=(10,5),layout='constrained',sharey=True)
limits=box(26.63,-.61,27.38,.61)
for ax,layer,title in zip(axes,[before[29],after[29]],['Original seam','Revised seam']):
    xy=layer[:,:,:2]
    for sign,color in [(-1,'#186c75'),(1,'#c77530')]:
        near=xy[(xy[:,:,0].max(axis=1)>26)&(np.min(abs(xy[:,:,1]),axis=1)<2)&(xy[:,:,1].mean(axis=1)*sign>0)]
        shape=unary_union([LineString(v).buffer(.20,quad_segs=32) for v in near]).intersection(limits)
        for poly in getattr(shape,'geoms',[shape]):
            if poly.geom_type=='Polygon':ax.fill(*poly.exterior.xy,color=color,alpha=.54,lw=0)
        ax.add_collection(LineCollection(near,colors=color,linewidths=1.5))
    ax.axvline(27,ls=':',color='#41484a',lw=1)
    p=crossings(layer,27);gap=abs(np.diff(p[:,1])[0])-.4
    ax.plot(p[:,0],p[:,1],'o',ms=4,color='#252d34')
    ax.set(xlim=(26.63,27.38),ylim=(-.61,.61),aspect='equal',xlabel='Radial position (mm)',title=f'{title}\n{abs(gap):.3f} mm '+('gap' if gap>0 else 'overlap'))
axes[0].set_ylabel('Across the slit (mm)')
fig.suptitle('Kevin  ·  Tread seam at layer 29',fontsize=17)
fig.supxlabel('Dots mark nozzle centers at radius 27 mm. Shading shows the nominal 0.400 mm bead in plan view.',fontsize=10)
fig.savefig(here/'seam-close-up.png',dpi=180)
