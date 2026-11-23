from pathlib import Path
import csv
import matplotlib.pyplot as plt
import numpy as np
p=Path(__file__).resolve().parent
def read(name):
    return np.genfromtxt(p/name,delimiter=',',names=True,dtype=None,encoding='utf8')
cube,robot=read('cube.csv.gz'),read('robot.csv.gz')
events=list(csv.DictReader((p/'events.csv').open()))
fig,axes=plt.subplots(3,2,figsize=(12,7.2),layout='constrained')
scores=[]
for row,name in enumerate(['W33','W34','W35']):
    c=cube[cube['trial']==name];s=robot[robot['trial']==name]
    e={r['event']:float(r['t_s']) for r in events if r['trial']==name}
    t=c['packet_s'];a=np.stack([c[k] for k in 'xyz'],axis=1)/8192
    ref=np.median(a[(t>=-7)&(t< -5)],axis=0)
    dev=np.linalg.norm(a-ref,axis=1)
    phases=[('baseline',-7,-5),('mount',-.8,e['spin_begin']),('spin',e['spin_begin'],e['spin_end']),('postspin',e['spin_end'],e['spin_end']+2),('late',e['spin_end']+2,e['landed'])]
    for phase,start,end in phases:
        sel=(t>=start)&(t<end)
        scores.append([name,phase,start,end,sel.sum(),np.sqrt(np.mean(dev[sel]**2)),np.quantile(dev[sel],.95),dev[sel].max(),len(set(c['tap'][sel]))])
    for ax,span in zip(axes[row],[(-2,3),(-2,24)]):
        visible=(t>=span[0])&(t<=span[1]);ax.scatter(t[visible],dev[visible],s=1.4,color='#255b85')
        ax.axvspan(e['spin_begin'],e['spin_end'],color='orange',alpha=.24)
        for rec in events:
            if rec['trial']==name and rec['event']=='dismount_step':ax.axvline(float(rec['t_s']),color='gray',alpha=.6,ls=':')
        ax.set(xlim=span,ylabel=f'{name} • Δaccel (g)')
        twin=ax.twinx();twin.plot(s['t_s'],s['pitch_deg'],color='#90264a',alpha=.6,lw=.8);twin.set(ylim=(-15,90),ylabel='SDK pitch (°)')
axes[0,0].set_title('Mount and commanded spin (shaded)')
axes[0,1].set_title('Full stand and dismount attempts')
for ax in axes[-1]: ax.set_xlabel('Seconds from detected stand')
fig.savefig(p/'cube-spin.png',dpi=160)
with (p/'phase-scores.csv').open('w') as f:
    writer=csv.writer(f,lineterminator='\n');writer.writerow('trial phase start_s end_s samples rms_g p95_g peak_g distinct_tap_counters'.split());writer.writerows(scores)
with (p/'spin-core.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('trial packets peak_delta_g rms_delta_g peak_packet_s peak_slot'.split())
    for name in ('W33','W34','W35'):
        c=cube[cube['trial']==name]; t=c['packet_s']
        a=np.column_stack([c[k] for k in 'xyz'])/8192
        a-=np.median(a[(t>=-7)&(t< -5)],axis=0)
        e={r['event']:float(r['t_s']) for r in events if r['trial']==name}
        use=(t>=e['spin_begin']+.1)&(t<e['spin_end']-.1)
        dev=np.linalg.norm(a[use],axis=1); j=np.argmax(dev)
        w.writerow([name,len(dev)//3,dev[j],np.sqrt(np.mean(dev**2)),t[use][j],c['sample'][use][j]])
