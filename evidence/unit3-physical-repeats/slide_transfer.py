"""Compare measured planar travel with the original and slower pad response at identical placements."""
from concurrent.futures import ThreadPoolExecutor
import csv
from pathlib import Path
import subprocess
import sys
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
out=here/'E'
def run(ms):
    args=[sys.executable,str(here/'oval_replay.py'),'--printed-fixture',sys.argv[1],'--cadence','--substeps','4',
          '--summary-only','--slide-summary','--pad-shelf-ms',str(ms)]
    result=subprocess.check_output(args,text=True)
    return [[ms,*row] for row in csv.reader(result.splitlines())]

if __name__=='__main__':
    with ThreadPoolExecutor(max_workers=2) as pool:
        rows=sum(pool.map(run,(8,32)),[])
    with (out/'slide-transfer.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['shelf_ms','dx_mm','dy_mm','initial_overlap_mm','first_contact_s','first_geom','yaw_deg','peak_g','taps','travel_mm'])
        w.writerows(rows)
    measured=np.genfromtxt(out/'tap-translation.csv',names=True,delimiter=',',dtype=None,encoding='utf8')
    first=measured[measured['center_s']==measured['center_s'][0]]['travel_mm']
    affine=measured[measured['center_s']==measured['center_s'][0]]['affine_travel_mm']
    fig,ax=plt.subplots(figsize=(7.3,4.2),layout='constrained')
    ax.axhspan(first.min(),first.max(),color='#c6e6ca',label='Projective travel')
    ax.axhspan(affine.min(),affine.max(),color='#e5be91',alpha=.7,label='Local affine travel')
    for ms,marker in [(8,'o'),(32,'^')]:
        group=[r for r in rows if r[0]==ms]
        ax.scatter([float(r[6]) for r in group],[float(r[9]) for r in group],label=f'{ms} ms pad response',marker=marker)
        nominal=next(r for r in group if float(r[1])==float(r[2])==0)
        ax.scatter(float(nominal[6]),float(nominal[9]),s=120,facecolors='none',edgecolors='black')
    ax.axvspan(14.148,18.013,color='#ddd5ec',alpha=.55,label='Measured yaw, two methods')
    ax.set(xlabel='Counterclockwise rotation (°)',ylabel='Planar cube travel (mm)',title='First contact · circles around nominal SDK placement')
    ax.legend(fontsize=8)
    fig.savefig(out/'slide-transfer.png',dpi=150)
