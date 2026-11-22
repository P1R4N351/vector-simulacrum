"""Locate excessive roll impacts in the previously registered rolling-tread replay."""
from pathlib import Path
import csv
import gzip
import sys
import numpy as np
import matplotlib.pyplot as plt
import mujoco as mj
from tread_speed import run, load_rows

here=Path(__file__).resolve().parent
parts=('shelf','tread','fork')
trace=[]

def observe(m,d):
    if d.time<2.45:return
    cube=m.body('cube').id
    loads=np.zeros((3,3))
    wrench=np.empty(6)
    for j,c in enumerate(d.contact):
        b1,b2=m.geom_bodyid[c.geom]
        if cube not in (b1,b2):continue
        other=b1 if b2==cube else b2
        part={0:0,m.body('tread').id:1,m.body('fork').id:2}[other]
        mj.mj_contactForce(m,d,j,wrench)
        loads[part]+=(1 if b2==cube else -1)*(wrench[:3]@c.frame.reshape(3,3))
    trace.append([d.time,*np.linalg.norm(loads,axis=1),np.linalg.norm(d.sensor('chip').data)/9.81,
                  *loads.sum(axis=0)])

def main():
    ref=here.parents[1]/'unit3-cube-accelerometer/resting.csv'
    r=np.genfromtxt(ref,delimiter=',',names=True)
    gain=np.linalg.norm([r[k+'_counts'].mean() for k in 'xyz'])/8192
    _,_,_,_,sensor=run(sys.argv[1],.00025,True,
                       sensor_overrides={'cube_chip_yaw_deg':180,'cube_accel_scale':[gain]*3},observer=observe)
    a=np.array(trace)
    with gzip.open(here/'impact-loads.csv.gz','wt') as f:
        np.savetxt(f,a,delimiter=',',fmt='%.6f',header='seconds,shelf_N,tread_N,fork_N,chip_g,net_fx_N,net_fy_N,net_fz_N',comments='')
    physical=np.genfromtxt(here/'cube.csv.gz',names=True,delimiter=',')
    zero=next(float(r['sec']) for r in load_rows('actions.csv') if r['action']=='roll cube')
    pt=physical['sec']-zero
    pnorm=np.linalg.norm(np.column_stack([physical[k] for k in 'xyz'])/8192,axis=1)
    st=np.repeat([t for t,b in sensor.messages],3)
    vals=np.vstack([np.frombuffer(b[2:],'<i2').reshape(3,3) for t,b in sensor.messages])/8192
    snorm=np.linalg.norm(vals,axis=1)
    fig,axes=plt.subplots(2,1,figsize=(9,5.8),sharex=True,layout='constrained')
    for j,name in enumerate(parts):
        axes[0].plot(a[:,0],a[:,j+1],label=name)
    axes[0].set(ylabel='Contact force on cube (N)',title='Rolling tread replay · resultant force by contact partner')
    axes[0].legend()
    axes[1].plot(st,snorm,'.',ms=3,label='Simulated packets')
    axes[1].plot(pt,pnorm,'.',ms=3,label='Physical packets')
    axes[1].legend()
    axes[1].set(xlim=(3,5.21),xlabel='Time after roll command (s)',ylabel='Packet magnitude (g)')
    fig.savefig(here/'impact-loads.png',dpi=150)
    with (here/'impact-windows.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(('from_s','to_s','sim_packet_max_g','real_packet_max_g','sim_packet_peak_time_s',
                    'max_shelf_N','max_tread_N','max_fork_N','raw_chip_max_g','raw_peak_time_s'))
        for low,high in [(3,3.5),(3.5,4),(4,4.5),(4.5,5),(5,5.207)]:
            samples=(a[:,0]>=low)&(a[:,0]<high)
            sim=(st>=low)&(st<high)
            hw=(pt>=low)&(pt<high)
            row=a[samples][a[samples,4].argmax()]
            w.writerow((low,high,snorm[sim].max(),pnorm[hw].max(),st[sim][snorm[sim].argmax()],
                        *a[samples,1:4].max(axis=0),row[4],row[0]))
if __name__=='__main__':main()
