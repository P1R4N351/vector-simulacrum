"""Compare actual free-body simulator packets with held-out stationary cube captures."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[2]))
from simulacrum import params
from simulacrum.robot import VectorSim


def simulate(values, sideways):
    sim=VectorSim(values=values,cube='free')
    if sideways:
        sim.data.joint('cube_root').qpos[3:]=[2**-.5,2**-.5,0,0]
    sim.advance(.6)
    first=len(sim.cube.messages)
    sim.advance(3)
    packets=[np.frombuffer(b[2:],'<i2').reshape(3,3) for _,b in sim.cube.messages[first:]]
    return np.linalg.norm(np.vstack(packets).mean(axis=0))/8192


def physical(path, pre_command=False):
    a=np.genfromtxt(path,delimiter=',',names=True)
    if pre_command:
        with (path.parent/'actions.csv').open() as f:
            zero=next(float(r['sec']) for r in csv.DictReader(f) if r['action']=='roll cube')
        a=a[(a['sec']>=zero-1)&(a['sec']<zero)]
    return np.linalg.norm([a[k].mean() for k in 'xyz'])/8192


if __name__=='__main__':
    values=params.values(params.load(model='vector_2'))
    poses=[('Pre-wheelstand',here.parent/'stationary.csv.gz',False,False),
           ('Repeated rest',here.parent/'stationary-repeat.csv.gz',False,False),
           ('After roll',here.parent/'R8/settled.csv.gz',True,False),
           ('Next approach',here/'cube.csv.gz',True,True)]
    fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
    with (here/'free-sensor.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('window','profile','measured_g','model_g','error_mg'))
        for label,config in [('Nominal scale',dict(values,cube_accel_scale=[1]*3)),('Shelf cube',values)]:
            pred={side:simulate(config,side) for side in (False,True)}
            error=[]
            for name,path,side,cut in poses:
                real=physical(path,cut)
                e=1000*(pred[side]-real)
                error.append(e)
                w.writerow((name,label,real,pred[side],e))
            ax.plot([p[0] for p in poses],error,'o-',label=label)
    ax.axhline(0,c='.5',lw=.8)
    ax.set(ylabel='Mean gravity vector magnitude error (mg)')
    ax.legend()
    fig.savefig(here/'free-sensor.png',dpi=150)
