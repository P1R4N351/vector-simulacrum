from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent
r=np.genfromtxt(p/'pose.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
h=np.genfromtxt(p/'camera.csv',delimiter=',',names=True)
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
names=sorted(set(r['trial']),key=lambda n:int(n[1:]))
jumps=[]
for name in names:
 s=r[r['trial']==name]
 assert np.all(s['origin']==s['origin'][-1])
 xy=np.c_[s['x_mm'],s['y_mm']]
 jumps.append(np.linalg.norm(xy[-1]-xy[-2]))
 if name=='W32':
  angle=s['yaw_rad'][-2]
  forward=(xy-xy[-2])@[np.cos(angle),np.sin(angle)]
  ax=axes[1];ax.plot(s['t_s'][-3:],forward[-3:],'ko-',label='SDK forward position')
  for key in ['front_x','rear_x']:
   disp=-(h[key]-h['static_x'])*48/127
   disp-=np.interp(s['t_s'][-2],h['t_s'],disp)
   ax.plot(h['t_s'],disp,'.-',label=key.split('_')[0]+' hub')
   print(key,np.interp(0,h['t_s'],disp))
  ax.axvspan(s['t_s'][-2],0,color='gray',alpha=.2)
  print('SDK forward',forward[-1])
axes[0].bar(names,jumps);axes[0].set_ylabel('SDK planar pose step (mm)')
axes[0].set_title('Last off to first on charger')
axes[1].set(xlim=(-.15,.15),xlabel='Seconds from first on-charger report',ylabel='Forward motion since last off (mm)',title='W32 • shelf camera and SDK')
axes[1].legend();fig.savefig(p/'contact.png',dpi=160)
print('median',np.median(jumps))
