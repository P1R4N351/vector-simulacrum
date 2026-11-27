"""Compare camera and encoder travel over the same physical pitch-to-flag interval."""
from pathlib import Path
import gzip
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
with gzip.open(here/'states.csv.gz','rt') as f:
    robot=np.genfromtxt(f,delimiter=',',names=True,dtype=None,encoding='utf8')
i=np.flatnonzero((robot['status']&4096)!=0)[0]
t=robot['receipt_s']-robot['receipt_s'][i]
deg=np.rad2deg(robot['pitch_rad'])
j=np.flatnonzero((deg[:i]>-5)&(deg[1:i+1]<=-5))[-1]+1
peak=t[j+np.argmin(deg[j:i+1])]
vel=(robot['left_mmps']+robot['right_mmps'])/2
enc=np.r_[0,-np.cumsum(np.diff(t)*(vel[:-1]+vel[1:])/2)]
enc-=np.interp(peak,t,enc)
h=np.genfromtxt(here/'dense-hubs.csv',delimiter=',',names=True,encoding='utf8',dtype=None)
manual=np.genfromtxt(here/'hubs.csv',delimiter=',',names=True)
scale=48/np.median(manual['rear_x']-manual['front_x'])
fig,ax=plt.subplots(figsize=(8,4.4),layout='constrained')
for direction in ('backward','forward'):
    r=h[h['direction']==direction]
    if direction=='backward': print('camera frames in interval:',np.count_nonzero((r['t_s']>=peak)&(r['t_s']<=0)))
    for hub in ('front','rear'):
        pos=(r[hub+'_x']-r['dock_x'])*scale
        pos-=np.interp(peak,r['t_s'],pos)
        cut=(r['t_s']>=peak-.1)&(r['t_s']<=.1)
        ax.plot(r['t_s'][cut],pos[cut],'.-' if direction=='backward' else ':',label=f'{hub}, {direction} template')
        print(direction,hub,round(np.interp(0,r['t_s'],pos),2))
ok=(t>=peak-.1)&(t<=.1)
ax.plot(t[ok],enc[ok],'k-',label='Integrated wheels')
ax.axvline(peak,color='.55',ls='--',lw=.8)
ax.axvline(0,color='.55',ls='--',lw=.8)
ax.set(xlabel='Seconds from first charger flag',ylabel='Reverse horizontal camera travel / wheel travel (mm)')
ax.legend(fontsize=8)
fig.savefig(here/'dense-travel.png',dpi=165)
print('SDK peak / encoder / scale:',peak,enc[i],scale)
r=h[h['direction']=='backward']
spacing=r['rear_x']-r['front_x']
mid=(r['front_x']+r['rear_x'])/2-r['dock_x']
pixels=np.interp(0,r['t_s'],mid)-np.interp(peak,r['t_s'],mid)
local_scales=48/np.interp([peak,0],r['t_s'],spacing)
print('Midpoint travel with each endpoint scale:',pixels*local_scales)
print('Clock shifted by one frame in either direction:',
      [scale*(np.interp(shift,r['t_s'],mid)-np.interp(peak+shift,r['t_s'],mid)) for shift in [-.035,.035]])
