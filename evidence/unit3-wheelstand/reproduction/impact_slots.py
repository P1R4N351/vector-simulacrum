"""Signed acceleration at physical and simulated wheelstand peaks."""
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
from acceleration_receipts import here, replay

def signed(x,rest):
    return (x-rest) @ (rest/np.linalg.norm(rest))

def window(series, j):
    assert 4<=j<len(series)-4
    return series[j-4:j+5]

def main(fixture):
    rows=[]
    gravity=[]
    fig, axes=plt.subplots(2,2,figsize=(10,6),layout='constrained',sharex=True,sharey='col')
    for col,name in enumerate(('W4','W8')):
        tr=replay.Trial(name)
        c=tr.cube
        assert np.array_equal(c['slot'],np.tile(np.arange(3),len(c)//3))
        t=c['sec']-tr.zero
        words=np.stack([c[k] for k in 'xyz'],axis=1)/8192
        rest=np.median(words[(t>=-.8)&(t<0)],axis=0)
        actual=signed(words,rest)
        mask=(t>=tr.late-.14)&(t<=tr.late+.8)
        peak=np.flatnonzero(mask)[np.argmax(abs(actual[mask]))]
        # Check for a receiver gap across any packet boundary spanned by the overlay.
        max_gap=np.max(np.diff(window(t,peak)))
        obs=window(actual,peak)
        for row in range(2):
            axes[row,col].plot(range(-4,5),obs,'ko-',label='Physical',zorder=5)
        rows.append((name,'physical',0,'physical',t[peak],peak%3,max_gap,*obs))
        v=replay.params.values(replay.params.load(model='vector_2'),{'tip_supports_driven':0})
        # If the only specific force were gravity, its magnitude would be one
        # in every orientation. Invert the configured chip calibration first.
        bias=np.array(v['cube_accel_bias_mg'])*.001
        force=(words-bias)/np.array(v['cube_accel_scale'])
        rest_norm=np.linalg.norm((rest-bias)/np.array(v['cube_accel_scale']))
        for j in range(peak-4,peak+5):
            norm=np.linalg.norm(force[j])
            gravity.append([name,j-peak,t[j],j%3,*force[j],norm,abs(norm-1),rest_norm])
        m=replay.build(v,fixture)
        m.opt.timestep=v['sim_timestep_ms']*.001/4
        baseline=np.array([0,0,v['cube_accel_scale'][2]])+np.array(v['cube_accel_bias_mg'])*.001
        for case,(s,o) in enumerate(((s,o) for s in (3.5,3.8) for o in (tr.early,tr.late)),1):
            start=s+tr.shift
            holder=[]
            def keep(run,t):
                if not holder: holder.append(run.sensor)
            replay.run(m,v,tr,start,o,'servo:30',17,on_step=keep)
            messages=holder[0].messages
            st=np.array([start+ts for ts,_ in messages])
            raw=np.stack([np.frombuffer(p,dtype='<i2',offset=2).reshape(3,3) for _,p in messages])/8192
            data=signed(raw,baseline).ravel()
            valid=np.repeat((st>=tr.late-.14)&(st<=tr.late+.8),3)
            indexes=(np.argmin(abs(st-t[peak]))*3+peak%3,np.flatnonzero(valid)[np.argmax(abs(data[valid]))])
            for r,j in enumerate(indexes):
                slots=window(data,j)
                rows.append((name,'sim',case,['physical time and slot','own largest magnitude'][r],st[j//3],j%3,.03,*slots))
                axes[r,col].plot(range(-4,5),slots,'.--',label=f'Case {case}',lw=1)
        axes[0,col].set_title(name+' · same receipt and slot index')
        axes[1,col].set_title(name+' · each trace at its own peak')
        axes[1,col].set_xlabel('Retained sample offset (nominally 10 ms per slot)')
    for ax in axes[:,0]: ax.set_ylabel('Signed acceleration from rest (g)')
    axes[1,1].legend(fontsize=7,ncol=2)
    fig.savefig(here/'impact-slots.png',dpi=150)
    with (here/'impact-gravity.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow('run offset packet_s slot x_g y_g z_g norm_g min_non_gravity_g resting_norm_g'.split())
        w.writerows(gravity)
    with (here/'impact-slots.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['run','source','case','alignment','center_packet_s','slot','max_receipt_gap_s',*[f'g_{j:+d}' for j in range(-4,5)]])
        w.writerows(rows)
if __name__=='__main__': main(sys.argv[1])
