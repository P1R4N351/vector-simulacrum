"""Compare pre-gap contact with the measured wrist, retaining the cube's three slots."""
import argparse
import csv

import matplotlib.pyplot as plt
import numpy as np

from wrist_signal import params, replay, root
from simulacrum.cube import MovementListener, engine_samples, packet


def states(times, slots):
    listener = MovementListener()
    result = []
    for t,group in zip(times,slots):
        for sample in group:
            listener.update(t,sample.astype(np.float32))
        result.append(listener.score)
    return np.array(result)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('printed_fixture')
    args=parser.parse_args()
    physical=np.loadtxt(root/'cube.csv',delimiter=',',skiprows=1).reshape(-1,3,5)
    t=physical[:,0,0]
    xyz=np.array([engine_samples(packet(0,g[:,2:].astype(int).tolist())) for g in physical])
    mean=xyz.mean(axis=1,keepdims=True)
    gravity=xyz[t<0].mean(axis=(0,1))
    traces={'Recorded':(t,states(t,xyz)),
            'Packet means only':(t,states(t,np.repeat(mean,3,axis=1))),
            'Within-packet variation only':(t,states(t,gravity+xyz-mean))}
    clock=np.arange(-.35,1.46,.001)
    enc=np.loadtxt(root/'encoder.csv',delimiter=',',skiprows=1)
    pitch=np.loadtxt(root/'dock-pitch.csv',delimiter=',',skiprows=1)
    packets=replay(args.printed_fixture,True,params.values(params.load(model='vector_2')),clock,
                   np.interp(clock,*enc[:,:2].T),np.interp(clock,*pitch[:,:2].T))
    sim_t=np.array([time for time,_ in packets])
    traces['Printed fork replay']=(sim_t,states(sim_t,[engine_samples(p) for _,p in packets]))
    end=t[(t>1.4)&(t<1.5)].item()
    # Split at the first and last encoder samples within 0.1 mm of the peak.
    top=enc[(enc[:,0]>=0)&(enc[:,0]<=end)]
    plateau=top[top[:,1]>=top[:,1].max()-.1,0]
    fixed = xyz.copy()
    first = np.searchsorted(t,plateau[0],side='right')
    fixed[first:] = xyz[first-1,-1]
    traces['Hold vector from peak']=(t,states(t,fixed))
    bounds=[0,plateau[0],plateau[-1],end]
    rows=[]
    fig,(ax,ex)=plt.subplots(2,1,sharex=True,figsize=(8,5),layout='constrained')
    for label,(times,scores) in traces.items():
        use=(times>=0)&(times<=end)
        ax.plot(times[use],scores[use],'.-',ms=3,label=label)
        for phase,lo,hi in zip(('Raise','Peak hold','Initial descent'),bounds,bounds[1:]):
            idx=np.searchsorted(times,[lo,hi],side='right')-1
            a,b=scores[idx]
            rows.append([label,phase,lo,hi,int(idx[1]-idx[0]),a,b,b-a])
    ex.plot(enc[:,0],enc[:,1],'.-',ms=3,color='.2')
    for panel in (ax,ex):
        panel.axvspan(*plateau[[0,-1]],alpha=.12,color='orange')
    ax.legend(fontsize=8); ax.set_ylabel('Movement score')
    ex.set(xlim=(0,end+.015),ylabel='Lift encoder (mm)',xlabel='Seconds from lift command')
    fig.savefig(root/'raise-buildup.png',dpi=165)
    with (root/'raise-buildup.csv').open('w') as f:
        w=csv.writer(f)
        w.writerow('source phase start_s end_s packets start_score end_score change'.split())
        w.writerows(rows)


if __name__=='__main__':
    main()
