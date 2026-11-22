from pathlib import Path
import csv,sys
import matplotlib.pyplot as plt
import numpy as np
p=Path(sys.argv[1])
with (p/'actions.csv').open() as f:
    actions=list(csv.DictReader(f))
def begin(name):return next(float(r['sec']) for r in actions if r['action']==name and r['kind']=='begin')
zero=begin('roll cube')
post=begin('passive post-stop capture')-zero
r=np.genfromtxt(p/'state.csv.gz',delimiter=',',names=True)
c=np.genfromtxt(p/'cube.csv.gz',delimiter=',',names=True)
t=c['sec']-zero; rt=r['sec']-zero
v=np.column_stack([c[k] for k in 'xyz'])/8192
before=v[(t>=-1)&(t<0)]
after=v[(t>=post+6)&(t<post+8)]
def axis(a):return 'XYZ'[np.argmax(abs(a))]+('+' if a[np.argmax(abs(a))]>0 else '-')
def metrics(a):return np.round(a.mean(0),4),np.round(a.std(0),4)
a,b=before.mean(0),after.mean(0)
window=(t>=0)&(t<=post+8)
print('pre/post',len(before),len(after),metrics(before),metrics(after),axis(a),axis(b),
      np.degrees(np.arctan2(np.linalg.norm(np.cross(a,b)),a@b)))
print('post stop',post,'last',t[-1],'peak',np.linalg.norm(v[window],axis=1).max(),'packets',window.sum()//3)
fig,axes=plt.subplots(3,1,figsize=(9,6.5),sharex=True,layout='constrained')
for i,k in enumerate('XYZ'):axes[0].plot(t[window],v[window,i],'.',ms=2,label=k)
axes[0].set_ylabel('Cube (g)');axes[0].legend(ncol=3)
axes[1].plot(rt,r['lift'],label='Lift (mm)')
axes[1].plot(rt,np.degrees(r['pitch']),label='Chassis pitch (°)');axes[1].legend()
axes[2].plot(rt,r['left'],label='Left');axes[2].plot(rt,r['right'],label='Right')
axes[2].set(xlabel='Seconds since roll command',ylabel='Wheel speed (mm/s)',xlim=(-.25,post+8));axes[2].legend()
for ax in axes:ax.axvline(post,c='.3',ls=':')
axes[0].set_title('Physical cube roll and passive observation after motor stop')
fig.savefig(p/'post-stop.png',dpi=150)
