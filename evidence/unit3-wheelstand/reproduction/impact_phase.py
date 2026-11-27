"""Separate contact-force, chip-frame and retained-slot peaks on fixed/30 replay."""
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
from acceleration_receipts import components, here, replay
from simulacrum.cube import CubeAccelerometer

class Frames(CubeAccelerometer):
    def _store(self, frame):
        self.recorded.append(frame)
        super()._store(frame)

def nearest(st,targets):
    i=np.searchsorted(st,targets).clip(1,len(st)-1)
    return i-(abs(st[i-1]-targets)<abs(st[i]-targets))

def main(fixture):
    scores=[]
    fig,axes=plt.subplots(1,2,figsize=(11,4.3),layout='constrained')
    for ax,name in zip(axes,('W4','W8')):
        tr=replay.Trial(name)
        c=tr.cube
        time=c['sec'][::3]-tr.zero
        words=np.stack([c[k] for k in 'xyz'],axis=1).reshape(-1,3,3)/8192
        rest=np.median(words[(time>=-.8)&(time<0)].reshape(-1,3),axis=0)
        lo,hi=tr.late-.14,tr.late+.8
        use=(time>=lo)&(time<=hi)
        observed=components(words[use],rest)[:,0]
        stamps=time[use]
        peak_idx=np.argmax(observed)
        ax.scatter(stamps-tr.late,observed,color='black',s=18,zorder=3,label='Physical receipts')
        v=replay.params.values(replay.params.load(model='vector_2'),{'tip_supports_driven':0})
        m=replay.build(v,fixture)
        dt=m.opt.timestep=v['sim_timestep_ms']*.001/4
        case=0
        all_preds=[]
        for release in (3.5,3.8):
            for onset in (tr.early,tr.late):
                case+=1
                start=release+tr.shift
                forces=[]
                def keep(r,t):
                    forces.append(r.d.sensor('cube_accel').data.copy()/(v['gravity_mmps2']*.001))
                replay.run(m,v,tr,start,onset,'servo:30',17,on_step=keep)
                times=np.arange(1,len(forces)+1)*dt
                raw=np.asarray(forces)
                baseline=np.array([0,0,v['cube_accel_scale'][2]])+np.array(v['cube_accel_bias_mg'])*.001
                direction=baseline/np.linalg.norm(baseline)
                raw_dev=abs(((raw-[0,0,1])*v['cube_accel_scale'])@direction)
                raw_peak=np.max(raw_dev[(start+times>=lo)&(start+times<=hi)])
                for fp in np.arange(4)*.0005:
                    chip=Frames(v,dt,seed=17,frame_phase_s=fp)
                    chip.recorded=[]
                    filtered_peak=0.
                    for sec,force in zip(times,raw):
                        chip.step(sec,force)
                        if lo<=start+sec<=hi:
                            filtered_peak=max(filtered_peak,abs((chip.filter.output[:3]-baseline)@direction))
                    frames=np.array(chip.recorded)/8192
                    ft=fp+np.arange(1,len(frames)+1)*chip.frame_s
                    dev=abs((frames-baseline)@direction)
                    chip_peak=max(dev[(start+ft>=lo)&(start+ft<=hi)])
                    for tp in np.arange(10)*.001:
                        ticks=tp+np.arange(1,1+int((times[-1]-tp+1e-12)/chip.tick_s))*chip.tick_s
                        ticks=ticks[:len(ticks)//3*3]
                        take=np.searchsorted(ft,ticks+1e-10,side='right')-1
                        assert take.min()>=0
                        if tp==0:
                            packet_words=np.stack([np.frombuffer(p, dtype='<i2', offset=2).reshape(3,3)
                                                   for _,p in chip.messages]).reshape(-1,3)
                            np.testing.assert_array_equal(frames[take]*8192,packet_words)
                        peaks=dev[take].reshape(-1,3).max(axis=1)
                        pt=start+ticks[2::3]
                        assert stamps[0]>=pt[0] and stamps[-1]<=pt[-1]
                        pred=peaks[nearest(pt,stamps)]
                        all_preds.append(pred)
                        scores.append([name,case,fp*1000,tp*1000,raw_peak,filtered_peak,chip_peak,max(pred),pred[peak_idx],
                                       observed[peak_idx],np.mean(abs(pred-observed))])
        low,high=np.min(all_preds,axis=0),np.max(all_preds,axis=0)
        ax.vlines(stamps-tr.late,low,high,color='teal',lw=3,label='Simulated phase and pose range')
        ax.set(title=name,xlabel='Seconds from last raised-lift sample',ylabel='Along resting gravity (g)')
    axes[0].legend(fontsize=8)
    fig.savefig(here/'impact-phase.png',dpi=160)
    with (here/'impact-phase.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow('run case chip_phase_ms app_phase_ms substep_peak_g filtered_peak_g chip_peak_g matched_peak_g at_physical_peak_g physical_peak_g mae_g'.split())
        w.writerows(scores)
if __name__=='__main__':main(sys.argv[1])
