"""Pose-only counterfactual on a continuous shelf through the first cliff event."""
from pathlib import Path
import sys
import mujoco
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[2]))
from simulacrum import mjcf, params
from simulacrum.hazards import Hazards

v=params.values(params.load(model="vector_2"))
m=mujoco.MjModel.from_xml_string(mjcf.build_mjcf(v))
d=mujoco.MjData(m)
supports=[g for g in range(m.ngeom) if m.geom_type[g]==mujoco.mjtGeom.mjGEOM_CYLINDER and ('wheel' in m.geom(g).name or 'tip_' in m.geom(g).name)]
assert len(supports)>=4
hazards=Hazards(v)
def replay(path, start, window=(-1,12)):
    s=np.genfromtxt(path,names=True,delimiter=',')
    rows=[]
    for rec in s:
        t=rec['sec']-start
        if not window[0]<=t<=window[1]:continue
        p=rec['pitch']
        d.qpos[:7]=[0,0,0,np.cos(p/2),0,-np.sin(p/2),0]
        mujoco.mj_forward(m,d)
        # All tread/axle support cylinder axes remain horizontal under pitch alone.
        d.qpos[2]=-min(d.geom_xpos[g,2]-m.geom_size[g,0] for g in supports)
        mujoco.mj_forward(m,d)
        ranges=np.array([d.sensor('cliff_'+name).data.item() for name in mjcf.CLIFF_NAMES])*1000
        hazards.update(ranges, v['gravity_mmps2'])
        rows.append([t,np.degrees(p),*ranges,int(hazards.cliff_any),int(bool(int(rec['status'])&16384))])
    return np.array(rows)

if __name__ == '__main__':
    a=replay(here/'S22B/state.csv.gz',31.691988)
    np.savetxt(here/'cliff-pitch.csv',a,header='roll_s,pitch_deg,front_left_mm,front_right_mm,rear_left_mm,rear_right_mm,sim_cliff,sdk_cliff',delimiter=',',comments='',fmt='%.5f')
    fig,(ax,bx)=plt.subplots(2,1,sharex=True,layout='constrained',figsize=(8,5))
    ax.plot(a[:,0],a[:,2],label='Front');ax.plot(a[:,0],a[:,4],label='Rear');ax.axhline(v['cliff_detect_range_mm'],ls=':',color='black',label='Model threshold');ax.legend();ax.set_ylabel('Simulated range (mm)')
    bx.step(a[:,0],a[:,7],where='post',label='SDK aggregate');bx.step(a[:,0],a[:,6],where='post',label='Flat shelf pitch replay',ls='--');bx.legend();bx.set(xlabel='Seconds from native roll',ylabel='Cliff flag',yticks=[0,1])
    fig.savefig(here/'cliff-pitch.png',dpi=150)
    for col in [6,7]:
        flagged=a[:,col]>0
        print(col,flagged.sum(),a[flagged,0][[0,-1]] if flagged.any() else [],'of',len(a))
    print('maximum rays',a[:,2:6].max(axis=0),'maximum pitch',a[:,1].max())
    print('at first flag',a[a[:,7]>0][0])

    # Choose a threshold on the Sunday roll and freeze it for two wheelstands.
    def mismatches(trace, threshold):
        rays=trace[:,2:6]
        prediction=((rays>threshold)|(rays<0)).any(axis=1)
        return int(np.count_nonzero(prediction != trace[:,7]))

    grid=np.arange(10,31,.25)
    best=min(grid,key=lambda h:(mismatches(a,h),abs(h-v['cliff_detect_range_mm'])))
    print('candidate',best)
    for name,start in [('S22B',31.691988),('W4',22.330433),('W8',21.147429)]:
        trace=a if name=='S22B' else replay(here.parent/name/'state.csv.gz',start)
        print(name,len(trace),int(sum(trace[:,7])),mismatches(trace,25),mismatches(trace,best))
