"""Predict elapsed receiver time from packet index on later physical repeats."""
import csv
import gzip
import sys
import matplotlib.pyplot as plt
import numpy as np
from packet_cadence import clocks,here

sys.path.insert(0,str(here.parent.parent))
from simulacrum.robot import VectorSim
from simulacrum import params

def predict(index,fast,slow,spacing):
    return index*fast+np.floor(index/spacing)*(slow-fast)

def main():
    rest=clocks('resting')[0]
    intervals=np.diff(rest)
    long=intervals>.15
    fast=intervals[~long].mean()
    slow=intervals[long].mean()
    spacing=np.diff(np.flatnonzero(long)).mean()
    sim=VectorSim(values=params.values(params.load(model='vector_2')),cube='free')
    sim.advance(3.)
    emitted=np.array([t for t,_ in sim.cube.messages])
    assert all(len(p)==20 and p[0]==2 for _,p in sim.cube.messages)
    cadence=np.median(np.diff(emitted))
    fig,axes=plt.subplots(2,3,figsize=(12,6),layout='constrained')
    with (here/'receipt-transfer.csv').open('w',newline='') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('run','packets','receiver_elapsed_s','model','elapsed_error_pct',
                    'max_abs_timeline_error_s','p95_abs_timeline_error_s'))
        for col,run in enumerate('AEF'):
            with gzip.open(here.parent/'unit3-physical-repeats'/run/'radio.csv.gz','rt') as stream:
                logs=[r for r in csv.DictReader(stream) if len(r['payload_hex'])==40 and r['payload_hex'].startswith('02')]
            actual=np.array([float(r['robot_uptime_s']) for r in logs])
            actual-=actual[0]
            n=np.arange(len(actual))
            axes[0,col].plot(n[1:],np.diff(actual)*1000,'.',ms=2)
            axes[0,col].set(title=f'Recording {run}',ylabel='Receiver interval (ms)')
            for label,estimate in [('Simulator emission',n*cadence),('Median pause model',predict(n,.03,.26,34)),
                                   ('Mean pause model',predict(n,fast,slow,spacing))]:
                err=estimate-actual
                axes[1,col].plot(n,err,label=label)
                w.writerow((run,len(n),actual[-1],label,100*err[-1]/actual[-1],max(abs(err)),np.quantile(abs(err),.95)))
            axes[1,col].set(xlabel='Received packet index',ylabel='Elapsed time error (s)')
    axes[1,0].legend(fontsize=8)
    fig.savefig(here/'receipt-transfer.png',dpi=160)
    print(fast,slow,spacing,cadence)
if __name__=='__main__':
    main()
