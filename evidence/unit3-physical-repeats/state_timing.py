"""Check the motion driving the fork against the chassis heading-rate measurements."""
from pathlib import Path
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[1]))
from simulacrum import params
from simulacrum.cube import CubeAccelerometer

out=here/'E'
force=np.load(sys.argv[1])
v=params.values(params.load(model='vector_2'))
chip=CubeAccelerometer(v,float(force['dt']),seed=17)
for i,g in enumerate(force['forces']):
    chip.step((i+1)*float(force['dt']),g)
original=[t+force['start'] for t in chip.taps.events if abs(t+force['start']-50.97782)<1]
corrected=np.atleast_1d(np.genfromtxt(out/'printed-contact-cadence-tap-times.csv',names=True)['candidate_time_s'])
r=np.genfromtxt(out/'robot.csv.gz',delimiter=',',names=True)
r=r[(r['receipt_s']>=45)&(r['receipt_s']<=54)]
t=r['receipt_s']
index=np.arange(len(t))
period,offset=np.polyfit(index,t,1)
regular=period*index+offset
measured=(r['gz'][1:]+r['gz'][:-1])/2
ang=np.diff(np.unwrap(r['yaw_rad']))
fig,axs=plt.subplots(2,1,figsize=(9,5.5),sharex=True,layout='constrained')
times=(t[1:]+t[:-1])/2
axs[0].plot(times,measured,'k.-',label='Reported gyro Z')
rows=[]
for label,clock in [('Receipt',t),('Uniform cadence',regular)]:
    speed=ang/np.diff(clock)
    axs[0].plot((clock[1:]+clock[:-1])/2,speed,'.-',label=label)
    for name,start,end in [('Earlier turn',46.2,47.4),('Contact turn',50.3,51.1)]:
        take=(times>=start)&(times<=end)
        rows.append([name,label,len(speed[take]),np.sqrt(np.mean((speed[take]-measured[take])**2))])
axs[0].set(ylabel='Yaw rate (rad/s)')
axs[0].legend(fontsize=8)
axs[1].plot(t,r['left_mmps'],'.-',label='Left wheel')
axs[1].plot(t,r['right_mmps'],'.-',label='Right wheel')
for label,events,color in [('Receipt replay tap',original,'#882244'),('Cadence replay tap',corrected,'#555599')]:
    for i,tm in enumerate(events):
        axs[1].axvline(tm,ls='--',color=color,label=label if i==0 else None)
axs[1].set(xlim=(50.25,51.25),xlabel='Seconds after undock command',ylabel='Wheel speed (mm/s)')
axs[1].legend(fontsize=8,ncol=2)
fig.savefig(out/'motion-clock.png',dpi=150)
with (out/'motion-clock.csv').open('w') as f:
    wr=csv.writer(f,lineterminator='\n')
    wr.writerow(['window','clock','intervals','gyro_difference_rms_rad_s'])
    wr.writerows(rows)
print(period,original,corrected,rows)
