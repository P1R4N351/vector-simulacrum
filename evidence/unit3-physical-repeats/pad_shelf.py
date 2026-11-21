"""Isolate shelf-pad compliance at two physics steps. Input: the registered printed-fork fixture."""
from concurrent.futures import ThreadPoolExecutor
import csv
import io
from pathlib import Path
import subprocess
import sys
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
out=here/'E'
def run(case):
    steps,ms=case
    command=[sys.executable,str(here/'oval_replay.py'),'--cadence','--printed-fixture',sys.argv[1],
             '--substeps',str(steps),'--nominal-only','--summary-only','--pad-shelf-ms',str(ms)]
    row=next(csv.reader(io.StringIO(subprocess.check_output(command,text=True))))
    return [1/steps,ms,float(row[3]),float(row[5]),float(row[6]),int(row[7])]

if __name__=='__main__':
    with ThreadPoolExecutor(max_workers=3) as pool:
        rows=list(pool.map(run,[(n,t) for n in (4,8) for t in (4,8,16,32)]))
    for step,ms,first,yaw,peak,taps in rows:
        if ms==8:
            suffix='sub4' if step==.25 else 'sub8-nominal'
            prior=next(r for r in csv.DictReader((out/f'printed-contact-cadence-{suffix}-replay.csv').open())
                       if float(r['cube_dx_mm'])==float(r['cube_dy_mm'])==0)
            print('8 ms explicit pair vs original:',step,yaw-float(prior['yaw_change_deg']),peak-float(prior['packet_peak_g']),taps-int(prior['packet_taps_in_window']))
    with (out/'pad-shelf.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['step_ms','pad_shelf_reference_ms','first_contact_s','yaw_deg','peak_g','taps']);w.writerows(rows)
    fig,axes=plt.subplots(1,3,figsize=(10,3.6),layout='constrained')
    for ax,i,title,obs in zip(axes,(3,4,5),('Yaw change (°)','Peak packet magnitude (g)','Packet taps'),((14.148,18.013),(1.078,1.078),(1,1))):
        for step in (.25,.125):
            group=np.array([row for row in rows if row[0]==step])
            ax.plot(group[:,1],group[:,i],'o-',label=f'{step} ms step')
        if obs[0]==obs[1]:ax.axhline(obs[0],color='green',ls='--')
        else:ax.axhspan(*obs,color='#d3e5d7')
        ax.set(title=title,xlabel='Pad–shelf time constant (ms)',xticks=[4,8,16,32])
    axes[1].legend(fontsize=8)
    fig.savefig(out/'pad-shelf.png',dpi=150)
