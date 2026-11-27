"""Compare cube acceleration components at physical receipt times during contact."""
import csv
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np
import replay

here = Path(__file__).parent
def components(slots, rest):
    direction = rest / np.linalg.norm(rest)
    delta = slots - rest
    parallel = delta @ direction
    transverse = delta - parallel[..., None] * direction
    return np.column_stack((np.max(abs(parallel), axis=1), np.max(np.linalg.norm(transverse, axis=2), axis=1)))

def main(fixture):
    fig, axes = plt.subplots(2,2,figsize=(11,6),layout='constrained')
    scores = []
    for col, name in enumerate(('W4','W8')):
        tr = replay.Trial(name)
        c = tr.cube
        assert np.array_equal(c['slot'], np.tile(np.arange(3), len(c)//3))
        time = c['sec'][::3]-tr.zero
        slots = np.stack([c[k] for k in 'xyz'],axis=1).reshape(-1,3,3)/8192
        rest = np.median(slots[(time>=-.8)&(time<0)].reshape(-1,3),axis=0)
        use = (time >= tr.late-.14) & (time <= tr.late+.8)
        physical = components(slots[use],rest)
        stamps = time[use]
        for axis,ax in enumerate(axes[:,col]):
            ax.scatter(stamps-tr.late,physical[:,axis],s=20,c='black',zorder=4,label='Physical receipts')
            for a,b in zip(stamps,stamps[1:]):
                if b-a>.15: ax.axvspan(a-tr.late,b-tr.late,color='.8',alpha=.5)
        for label,driven,cap,color in [('fixed/30',0,30,'tab:blue'),('driven/30',1,30,None),('fixed/60',0,60,None),('driven/60',1,60,'darkorange')]:
            v = replay.params.values(replay.params.load(model='vector_2'),{'tip_supports_driven':driven})
            model = replay.build(v,fixture)
            model.opt.timestep = v['sim_timestep_ms']*.001/4
            baseline = np.array([0,0,v['cube_accel_scale'][2]])+np.array(v['cube_accel_bias_mg'])*.001
            for case,(s,o) in enumerate(((s,o) for s in (3.5,3.8) for o in (tr.early,tr.late)),1):
                start = s+tr.shift
                holder = []
                def keep(run,t):
                    if not holder: holder.append(run.sensor)
                replay.run(model,v,tr,start,o,f'servo:{cap}',17,on_step=keep)
                messages=holder[0].messages
                st=np.array([start+t for t,_ in messages])
                data=np.stack([np.frombuffer(p,dtype='<i2',offset=2).reshape(3,3) for _,p in messages])/8192
                sim=components(data,baseline)
                for offset in (-.03,0,.03):
                    targets=stamps+offset
                    valid=(targets>=st[0])&(targets<=st[-1])
                    assert valid.all(), "All configurations must score identical physical observations"
                    ix=np.searchsorted(st,targets[valid]).clip(1,len(st)-1)
                    ix-=abs(st[ix-1]-targets[valid])<abs(st[ix]-targets[valid])
                    matched=sim[ix]
                    err=np.abs(matched-physical[valid]).mean(axis=0)
                    scores.append([name,label,case,round(offset*1000),sum(valid),*err,*matched.max(axis=0),*physical[valid].max(axis=0)])
                    if offset==0 and color is not None:
                        for k,ax in enumerate(axes[:,col]):
                            ax.scatter(stamps[valid]-tr.late,matched[:,k],s=9,color=color,alpha=.5,
                                       label=label if case==1 else None)
        axes[0,col].set_title(name)
        axes[1,col].set_xlabel('Seconds from last raised-lift sample')
    for i,title in enumerate(('Along resting gravity','Across resting gravity')):
        axes[i,0].set_ylabel(title+' (g)')
    axes[0,0].legend(fontsize=8)
    fig.savefig(here/'acceleration-receipts.png',dpi=155)
    with (here/'acceleration-receipts.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow('run model case offset_ms receipts mae_parallel_g mae_transverse_g sim_peak_parallel_g sim_peak_transverse_g physical_peak_parallel_g physical_peak_transverse_g'.split())
        w.writerows(scores)
if __name__=='__main__': main(sys.argv[1])
