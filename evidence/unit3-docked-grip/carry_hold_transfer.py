"""Compare packet-scale vibration in the failed grip with steady, reported carries."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent


def rows(path):
    with (gzip.open if path.suffix=='.gz' else open)(path,'rt') as f:
        return list(csv.DictReader(f))


def select(states, packets):
    # Include state samples bracketing the full 200 ms window around receipt.
    times = states[:,0]
    out=[]
    heights=[]
    for group in packets.reshape(-1,3,5):
        t=group[0,0]
        assert np.all(group[:,0]==t) and np.all(group[:,1]==[0,1,2])
        lo,hi=np.searchsorted(times,[t-.1,t+.1])
        if lo==0 or hi==len(times):
            continue
        window=states[lo-1:hi+1]
        if (np.diff(window[:,0]).max()>.15 or not window[:,4].all()
                or abs(window[:,2:4]).max()>1 or np.ptp(window[:,1])>.2):
            continue
        out.append(group)
        heights.append(np.median(window[:,1]))
    return np.array(out), heights


def main():
    sets={}
    p=root.parent/'unit3-cube-accelerometer'
    states=np.loadtxt(p/'robot.csv',delimiter=',',skiprows=1)
    packets=np.loadtxt(p/'pickup.csv',delimiter=',',skiprows=1)
    sets['Earlier pickup']=select(np.column_stack([states[:,0],states[:,1],states[:,3:5],states[:,5]==1]),
                                np.column_stack([packets[:,0]/1e9,packets[:,2],packets[:,4:7]]))
    p=root.parent/'unit3-wheelstand/R5'
    states=np.loadtxt(p/'state.csv.gz',delimiter=',',skiprows=1)
    packets=np.loadtxt(p/'cube.csv.gz',delimiter=',',skiprows=1)
    sets['R5 pickup']=select(np.column_stack([states[:,0],states[:,3:6],(states[:,6].astype(int)&2)!=0]),
                             packets[:,[0,2,4,5,6]])
    dock=np.loadtxt(root/'cube.csv',delimiter=',',skiprows=1).reshape(-1,3,5)
    sets['Failed hold']=dock[(dock[:,0,0]>=1.15001)&(dock[:,0,0]<=1.37309)]
    sets['Docked quiet']=dock[(dock[:,0,0]<0)|(dock[:,0,0]>2.5)]
    encoder=np.loadtxt(root/'encoder.csv',delimiter=',',skiprows=1)
    for key in ('Failed hold','Docked quiet'):
        groups=sets[key]
        sets[key]=(groups,np.interp(groups[:,0,0],encoder[:,0],encoder[:,1],left=np.nan,right=np.nan))
    fig,ax=plt.subplots(figsize=(7,4),layout='constrained')
    data=[]
    for label,(groups,heights) in sets.items():
        assert len(groups)>0,label
        steps=np.abs(np.diff(np.linalg.norm(groups[:,:,2:]/8192,axis=2),axis=1))*1000
        peak=np.max(steps,axis=1)
        ax.step(np.sort(peak),np.arange(1,len(peak)+1)/len(peak),where='post',label=f'{label} ({len(peak)})')
        data.append([label,len(groups),np.median(steps),np.median(peak),*np.percentile(peak,[90,100]),np.nanmedian(heights)])
    ax.set(xscale='log',xlabel='Largest within-packet magnitude change (mg)',ylabel='Fraction of packets at or below value')
    ax.legend(fontsize=8)
    fig.savefig(root/'carry-hold-transfer.png',dpi=165)
    with (root/'carry-hold-transfer.csv').open('w') as f:
        w=csv.writer(f)
        w.writerow('source packets median_step_mg median_packet_max_mg p90_packet_max_mg max_mg median_available_lift_mm'.split())
        w.writerows(data)


if __name__=='__main__':
    main()
