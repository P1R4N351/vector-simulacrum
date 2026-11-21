"""Sweep the two sensor clocks over the same nominal contact trajectory for each fork."""
import argparse
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[1]))
from simulacrum import params
from simulacrum.cube import CubeAccelerometer

def sweep(path, geometry, values):
    source=np.load(path)
    dt=float(source['dt'])
    start=float(source['start'])
    rows=[]
    for frame_ms in np.arange(0,values['cube_accel_frame_period_ms'],.5):
        for tick_ms in np.arange(0,values['cube_tick_ms'],.5):
            chip=CubeAccelerometer(values,dt,seed=17,frame_phase_s=frame_ms*.001,tick_phase_s=tick_ms*.001)
            for i,force in enumerate(source['forces']):
                chip.step((i+1)*dt,force)
            in_window=lambda t: abs(t+start-50.97782)<1
            packets=[p for t,p in chip.messages if in_window(t)]
            slots=np.stack([np.frombuffer(p,'<i2',offset=2).reshape(3,3) for p in packets])
            peak=np.linalg.norm(slots/8192,axis=2).max()
            rows.append((geometry,frame_ms,tick_ms,peak,sum(in_window(t) for t in chip.taps.events)))
    return rows

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('boxes',type=Path)
    parser.add_argument('printed',type=Path)
    args=parser.parse_args()
    values=params.values(params.load(model='vector_2'))
    rows=sweep(args.boxes,'Three boxes',values)+sweep(args.printed,'Printed fork',values)
    out=here/'E'
    with (out/'phase-sweep.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['geometry','frame_phase_ms','tick_phase_ms','peak_g','tap_candidates'])
        w.writerows(rows)
    cube=np.genfromtxt(out/'cube.csv.gz',names=True,delimiter=',')
    window=np.abs(cube['receipt_s']-50.97782)<1
    measured=np.linalg.norm([cube[k][window] for k in ('x_counts','y_counts','z_counts')],axis=0).max()/8192
    fig,axes=plt.subplots(1,2,figsize=(9,3.8),layout='constrained',sharey=True)
    for ax,geometry in zip(axes,['Three boxes','Printed fork']):
        rr=[r for r in rows if r[0]==geometry]
        data=np.array([r[1:] for r in rr])
        baseline=np.genfromtxt(out/('first-contact-replay.csv' if geometry=='Three boxes' else 'printed-contact-replay.csv'),delimiter=',',names=True,dtype=None,encoding='utf8')
        nominal=baseline[(baseline['cube_dx_mm']==0)&(baseline['cube_dy_mm']==0)][0]
        assert np.isclose(data[0,2],nominal['packet_peak_g']) and data[0,3]==nominal['packet_taps_in_window']
        for phase in np.unique(data[:,0]):
            part=data[data[:,0]==phase]
            ax.plot(part[:,1],part[:,2],'.-',label=f'{phase:g} ms')
        ax.axhline(measured,ls='--',color='k',label='Physical')
        ax.set(title=geometry,xlabel='Cube app tick phase (ms)')
        print(geometry, 'peak extrema',data[:,2].min(),data[:,2].max(),'taps',np.unique(data[:,3],return_counts=True),'observed',measured)
    axes[0].set_ylabel('Peak packet magnitude (g)')
    axes[1].legend(fontsize=8,title='Sensor frame phase')
    fig.savefig(out/'phase-sweep.png',dpi=150)
