"""Summarize the physical cube during an action and the subsequent settling window."""
import csv
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np


def main(folder, action):
    with (folder/'actions.csv').open() as f:
        rows=list(csv.DictReader(f))
    start=next(float(r['sec']) for r in rows if r['action']==action and r['kind']=='begin')
    cube=np.genfromtxt(folder/'cube.csv.gz',names=True,delimiter=',')
    state=np.genfromtxt(folder/'state.csv.gz',names=True,delimiter=',')
    t=cube['sec']-start
    g=np.column_stack([cube[k] for k in 'xyz'])/8192
    norm=np.linalg.norm(g,axis=1)
    mask=(t>=0)&(t<=24)
    print(action,'range',t[[0,-1]],'slots 0–24 s',sum(mask),'min/max',norm[mask].min(),norm[mask].max())
    baseline=g[(t>=-1)&(t<0)].mean(axis=0)
    for sec in [2,6,10,16,22]:
        sample=g[(t>=sec)&(t<sec+1)]
        if len(sample):
            avg=sample.mean(axis=0)
            print(sec,len(sample),np.round(avg,3),'angle',np.degrees(np.arctan2(np.linalg.norm(np.cross(baseline,avg)),baseline@avg)),'scatter',np.std(sample,axis=0).max())
    print('tap changes',t[1:][np.diff(cube['tap'])!=0])
    fig,axes=plt.subplots(3,1,figsize=(9,6),sharex=True,layout='constrained')
    for j,name in enumerate('xyz'):
        axes[0].plot(t,g[:,j],'.',ms=2,label=name.upper())
    axes[0].set_ylabel('Cube axes (g)')
    axes[0].legend(ncol=3)
    axes[1].plot(t,norm,'.',ms=2)
    axes[1].set_ylabel('Magnitude (g)')
    axes[2].plot(state['sec']-start,state['lift'],label='Lift (mm)')
    axes[2].plot(state['sec']-start,np.degrees(state['pitch']),label='Pitch (deg)')
    axes[2].legend()
    axes[2].set(xlim=(-2,24),xlabel=f'Seconds after {action} command')
    fig.savefig(folder/'contact.png',dpi=150)


if __name__=='__main__':
    main(Path(sys.argv[1]),sys.argv[2])
