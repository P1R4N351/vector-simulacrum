"""Bracket the camera and telemetry host clocks against one observer via persistent SSH."""
import argparse
import csv
from pathlib import Path
import shlex
import subprocess
import time

import numpy as np
import matplotlib.pyplot as plt

REMOTE = """import sys,time
for line in sys.stdin:
    a=time.monotonic_ns()
    wall=time.time_ns()
    b=time.monotonic_ns()
    print(a,wall,b,flush=True)
"""


def connect(host, via=None):
    cmd = ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',host,shlex.join(['python3','-u','-c',REMOTE])]
    if via:
        cmd = ['ssh','-o','BatchMode=yes',via,shlex.join(cmd)]
    return subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)


def sample(proc):
    before=time.time_ns()
    proc.stdin.write('sample\n')
    proc.stdin.flush()
    a,wall,b=map(int,proc.stdout.readline().split())
    after=time.time_ns()
    return (before,after,a,wall,b)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('camera_host')
    parser.add_argument('telemetry_host')
    parser.add_argument('output',type=Path)
    parser.add_argument('--telemetry-via')
    args=parser.parse_args()
    pairs=[]
    camera=connect(args.camera_host)
    telemetry=connect(args.telemetry_host,args.telemetry_via)
    try:
        for proc in (camera,telemetry):
            sample(proc)  # Exclude connection setup.
        for i in range(60):
            # Reverse query order on alternating rounds.
            if i%2:
                b=sample(telemetry)
                a=sample(camera)
            else:
                a=sample(camera)
                b=sample(telemetry)
            pairs.append((a,b))
            time.sleep(.15)
    finally:
        for proc in (camera,telemetry):
            proc.stdin.close()
            proc.wait(timeout=8)
    args.output.mkdir(exist_ok=True,parents=True)
    with (args.output/'host-clock.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['round','host','sent_unix_ns','received_unix_ns','remote_before_mono_ns','remote_unix_ns','remote_after_mono_ns'])
        for i,pair in enumerate(pairs):
            for name,record in zip(('camera','telemetry'),pair):
                w.writerow([i,name,*record])
    data=np.array(pairs,dtype=np.int64)
    low=(data[:,:,3]-data[:,:,1])*1e-6
    high=(data[:,:,3]-data[:,:,0])*1e-6
    def bound(sl):
        l=low[sl].max(axis=0)
        h=high[sl].min(axis=0)
        if np.any(l>h):raise RuntimeError('Inconsistent constant-clock assumption')
        return l[0]-h[1],h[0]-l[1]
    fig,ax=plt.subplots(figsize=(9,3.5),layout='constrained')
    ax.vlines(np.arange(len(data)),low[:,0]-high[:,1],high[:,0]-low[:,1],color='.55',lw=1)
    for sl in [slice(0,30),slice(30,60),slice(None)]:
        print(sl,'camera minus telemetry wall offset (ms):',bound(sl))
    lo,hi=bound(slice(None))
    ax.axhspan(lo,hi,color='tab:blue',alpha=.3,label='Intersection across all probes')
    ax.set(xlabel='Probe pair',ylabel='Camera minus telemetry clock (ms)')
    ax.legend()
    fig.savefig(args.output/'host-clock.png',dpi=150)


if __name__=='__main__':
    main()
