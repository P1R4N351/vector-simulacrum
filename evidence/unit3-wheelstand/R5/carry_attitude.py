"""Compare cube-frame gravity before and after the four reverse moves."""
from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
def read(path):
    return np.genfromtxt(path,delimiter=',',names=True)
cube=read(here/'cube.csv.gz')
robot=read(here/'state.csv.gz')
vec=np.column_stack([cube[k] for k in 'xyz'])/8192
def tilt(v):
    return np.degrees(np.arctan2(np.linalg.norm(v[..., :2],axis=-1),v[...,2]))
def direction_change(a,b):
    return np.degrees(np.arctan2(np.linalg.norm(np.cross(a,b)),a@b))
with (here/'carry-attitude.csv').open('w') as out:
    w=csv.writer(out,lineterminator='\n')
    w.writerow(['window','start_s','end_s','samples','mean_x_g','mean_y_g','mean_z_g','tilt_deg','norm_g','vector_rms_scatter_g','delta_from_pre_reverse_deg','max_abs_wheel_mmps'])
    baseline=None
    for name,lo,hi in [('Pre-reverse hold',29,31.5),('Post-reverse hold',38,39.5),('Released',42.5,45.5)]:
        v=vec[(cube['sec']>=lo)&(cube['sec']<hi)]
        states=robot[(robot['sec']>=lo)&(robot['sec']<hi)]
        m=v.mean(axis=0)
        if baseline is None:baseline=m
        w.writerow([name,lo,hi,len(v),*m,tilt(m),np.linalg.norm(m),np.sqrt(np.mean(np.sum((v-m)**2,axis=1))),direction_change(baseline,m),max(abs(states['left']).max(),abs(states['right']).max())])
fig,ax=plt.subplots(2,1,sharex=True,figsize=(9,5),layout='constrained')
ax[0].scatter(cube['sec'],tilt(vec),s=2,alpha=.45)
ax[0].set(ylabel='Apparent cube tilt (°)',ylim=(-2,100))
ax[1].plot(robot['sec'],robot['lift'],label='Lift (mm)')
ax[1].plot(robot['sec'],robot['left'],label='Left wheel (mm/s)')
ax[1].plot(robot['sec'],robot['right'],label='Right wheel (mm/s)',alpha=.65)
for a in ax:
    for lo,hi in [(29,31.5),(38,39.5),(42.5,45.5)]:a.axvspan(lo,hi,color='green',alpha=.12)
ax[1].set(xlim=(23,47),xlabel='Seconds after undock command')
ax[1].legend()
fig.savefig(here/'carry-attitude.png',dpi=150)

# Transfer original upright-only fits to a carry tipped along another sensor axis.
cal=here.parents[1]/'unit3-cube-accelerometer'
r=read(cal/'resting.csv')
rest=np.array([r[k+'_counts'].mean()/8192 for k in 'xyz'])
ideal_z=np.sqrt(1-rest[:2]@rest[:2])
with (here/'gravity-transfer.csv').open('w') as out:
    w=csv.writer(out,lineterminator='\n')
    w.writerow(['candidate','pre_reverse_norm_error_mg','post_reverse_norm_error_mg'])
    for label,scale,offset in [('Nominal',1.,0.),('Common gain',np.linalg.norm(rest),0.),
                               ('Z offset',1.,np.array([0,0,rest[2]-ideal_z])),
                               ('Z gain',np.array([1,1,rest[2]/ideal_z]),0.)]:
        row=[]
        for lo,hi in [(29,31.5),(38,39.5)]:
            m=vec[(cube['sec']>=lo)&(cube['sec']<hi)].mean(axis=0)
            row.append(1000*(np.linalg.norm((m-offset)/scale)-1))
        w.writerow([label,*row])
