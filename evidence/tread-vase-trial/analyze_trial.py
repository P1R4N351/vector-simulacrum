"""Inspect modal Marlin XYZE moves emitted by slice_trial.py (absolute XYZE)."""
from pathlib import Path
import re,csv,sys
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import trimesh

root=Path(sys.argv[1]); out=Path(__file__).resolve().parent
def parse(path):
    state=np.zeros(4); layer=-1; drawn={}; travel=[]
    for line in path.read_text().splitlines():
        if line.startswith(';LAYER:'): layer=int(line[7:]);drawn[layer]=[]
        cmd=line.split(';')[0].split()
        if not cmd or cmd[0] not in ('G0','G1','G92'):continue
        move=state.copy()
        for word in cmd[1:]:
            if word[0] in 'XYZE':move['XYZE'.index(word[0])]=float(word[1:])
        if layer>=0 and cmd[0]!='G92' and np.linalg.norm(move[:3]-state[:3])>0:
            if move[3]>state[3]:
                drawn[layer].append([state[:3].copy(),move[:3].copy()])
            else:travel.append((layer,move.copy(),state.copy()))
        state=move
    return {k:np.array(v)-[50,50,0] for k,v in drawn.items()}, travel

def hits(segments,angle):
    u=np.array([np.cos(angle),np.sin(angle)]); n=np.array([-u[1],u[0]])
    aa,bb=segments[:,0],segments[:,1]; a,b=aa[:,:2]@n,bb[:,:2]@n
    ok=(a>0)!=(b>0)
    p=aa[ok]+(bb-aa)[ok]*(a[ok]/(a-b)[ok])[:,None]
    r=p[:,:2]@u; pairs=np.column_stack([r,p[:,2]])[r>0]
    return pairs[np.argsort(pairs[:,0])]

def draw(ax,s,color):
    ax.add_collection(LineCollection(s[:,:,:2],colors=color,linewidths=1));ax.autoscale();ax.set_aspect('equal')

results=[]
for file in sorted(root.glob('*.gcode')):
    layers,trav=parse(file); rows=layers[29]
    # Exclude the slit ends; sample the two strands at matched azimuths.
    angles=np.deg2rad(np.arange(1.05,359,.1)); pairs=[hits(rows,a) for a in angles]
    matched=[(a,p) for a,p in zip(angles,pairs) if len(p)==2]
    radii=np.array([p[0,0] for a,p in matched])
    teeth=len(find_peaks(-radii,prominence=.15,distance=10)[0])
    results.append(dict(slit_mm=file.stem[5:],midlayer_inner_teeth=teeth,
        interior_nonextruding_moves=sum(0<k<57 for k,m,s in trav),matched_rays=len(matched),
        min_inner_radius_mm=radii.min(),max_inner_radius_mm=radii.max()))
    if file.stem=='slit-0.05':
        chosen=layers; chosen_trav=trav
with (out/'slit-sweep.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=results[0]);w.writeheader();w.writerows(results)
angles=np.deg2rad(np.arange(1.05,359,.5))
pairs=[]; missing=0
for k in range(2,56):
    for t in angles:
        before=hits(chosen[k-1],t); here=hits(chosen[k],t); after=hits(chosen[k+1],t)
        if any(len(x)!=2 for x in (before,here,after)):missing+=1;continue
        # Both possible neighbors of the inner bead in the outer stack.
        zo=here[1,1]; zi=here[0,1]
        below,above=(zo,after[1,1]) if zo<=zi else (before[1,1],zo)
        pairs.append((k,np.rad2deg(t),zi-below,above-zi,here[1,0]-here[0,0]))
d=np.array(pairs)
mesh=trimesh.load_mesh(root/'slit-0.00.stl')
fig,axes=plt.subplots(1,3,figsize=(13,4),layout='constrained')
s=chosen[29]
for ax in axes[:2]:
    for c in mesh.section([0,0,1],[0,0,5.9]).discrete:ax.plot(c[:,0],c[:,1],color='#989797',lw=.8)
    draw(ax,s,'#087789')
axes[0].set(title='Cura extrusion at the toothed band',xlabel='x (mm)',ylabel='y (mm)')
axes[1].set(xlim=(25.8,28.5),ylim=(-3,3),title='Radial slit on the right',xlabel='x (mm)',ylabel='y (mm)')
j=d[:,0]==29
axes[2].plot(d[j,1],d[j,2],color='#087789',label='To outer bead below')
axes[2].plot(d[j,1],d[j,3],color='#b26523',label='To outer bead above')
axes[2].set(title='Local vertical staggering',xlabel='Azimuth (degrees)',ylabel='Height difference (mm)');axes[2].legend()
fig.savefig(out/'toolpath.png',dpi=160)
# On seam, intersect along y at radial mid-band. Use segments near the positive x axis.
q=s[:,0];v=s[:,1];ref=27.0
mask=((q[:,0]>ref)!=(v[:,0]>ref))
ends=q[mask]+(v-q)[mask]*((ref-q[mask,0])/(v-q)[mask,0])[:,None]
print('seam y at x=27', sorted(y for y in ends[:,1] if abs(y)<1))
with (out/'inner-teeth-by-layer.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['cura_layer','inner_teeth','two_strand_rays','total_rays'])
    for k in range(1,57):
        inner=[p[0,0] for t in angles if len(p:=hits(chosen[k],t))==2]
        w.writerow([k,len(find_peaks(-np.array(inner),prominence=.15)[0]),len(inner),len(angles)])
print('z sum min/max', np.min(d[:,2]+d[:,3]),np.max(d[:,2]+d[:,3]))
print('sampled pairs,missing',len(d),missing,'minmax',d.min(0),d.max(0),'z sum range',np.ptp(d[:,2]+d[:,3]),'median sum',np.median(d[:,2]+d[:,3]))
print('radial separation > width fraction',np.mean(d[:,-1]>.405),'negative deposited dz',min(np.min(v[:,1,2]-v[:,0,2]) for v in chosen.values()),'nonextruding lowering',[(k,m[2]-s[2]) for k,m,s in chosen_trav if m[2]<s[2]])
