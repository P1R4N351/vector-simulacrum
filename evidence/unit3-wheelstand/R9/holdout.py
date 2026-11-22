"""Test the previously frozen rolling-tread fixture against a visually failed approach."""
import csv
from pathlib import Path
import sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/'R8'))
from tread_speed import run,load_rows

cube=next(r for r in load_rows('roll-poses.csv',here) if r['source']=='robot_observed_object' and float(r['sec'])<0)
raw=Rotation.from_quat([float(cube[k]) for k in ('qw','qx','qy','qz')],scalar_first=True)
vertical=raw.inv().apply([0,0,1])
axis=np.eye(3)[np.argmax(abs(vertical))]*np.sign(vertical[np.argmax(abs(vertical))])
correction,_=Rotation.align_vectors([[0,0,1]],[raw.apply(axis)])
rotation=correction*raw
t,tilt,hit,speed,sensor=run(sys.argv[1],.00025,True,here,rotation)
cube_data=np.genfromtxt(here/'cube.csv.gz',names=True,delimiter=',')
zero=next(float(r['sec']) for r in load_rows('actions.csv',here) if r['action']=='roll cube')
time=cube_data['sec']-zero
mask=(time>0)&(time<t[-1])
measured=np.linalg.norm(np.column_stack([cube_data[k] for k in 'xyz'])/8192,axis=1)
packets=[(s,np.frombuffer(b[2:],'<i2').reshape(3,3)/8192) for s,b in sensor.messages]
at=np.repeat([s for s,v in packets],3)
values=np.linalg.norm(np.vstack([v for s,v in packets]),axis=1)
# Drop the first half second of simulated shelf settling from event and peak scoring.
print('contact seconds:',t[hit][:1], 'peak tilt',max(tilt),'end tilt',tilt[-1],
      'taps', [s for s in sensor.taps.events if s>.5], 'peak g sim/physical', max(values[at>.5]), max(measured[mask&(time>.5)]))
fig,ax=plt.subplots(2,1,figsize=(8,4.8),sharex=True,layout='constrained')
ax[0].plot(at,values,'.',ms=2,label='Frozen rolling-tread model')
ax[0].plot(time[mask],measured[mask],'.',ms=2,label='Physical cube');ax[0].legend();ax[0].set_ylabel('Cube acceleration (g)')
ax[1].plot(t,tilt);ax[1].set(xlabel='Seconds after roll command',ylabel='Simulated tilt (°)',xlim=(0,19.2))
fig.savefig(here/'holdout.png',dpi=150)
