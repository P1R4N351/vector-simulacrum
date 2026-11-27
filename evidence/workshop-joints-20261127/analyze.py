import csv,sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
def read(path):
    with open(path) as f: return list(csv.DictReader(f))
files=[('Workshop A · prior',read(sys.argv[1]),'#537399'),('Workshop B · fresh',read('joints.csv'),'#c06229')]
fig,axes=plt.subplots(1,3,figsize=(11,3.6),layout='constrained')
with open('transits.csv','w') as f:
    w=csv.writer(f);w.writerow(['unit','joint','direction','elapsed_ms'])
    for label,rows,color in files:
        commands=[(i,r) for i,r in enumerate(rows) if r['event'] in ('head','lift')]
        for j,(i,r) in enumerate(commands):
            if r['event']=='head' and (not j or commands[j-1][1]['event']!='head' or float(r['target']) not in (-8,22)): continue
            seg=[a for a in rows[i+1:commands[j+1][0] if j+1<len(commands) else None] if a['event']=='state']
            if not seg: continue
            head=r['event']=='head'
            if label.startswith('Workshop A') and (not head or float(r['condition'])!=0): continue
            sign=1 if float(r['target'])>0 else -1
            t=np.array([float(a['s']) for a in seg]); x=np.array([float(a['head_deg' if head else 'lift_mm']) for a in seg])*sign
            lo,hi=((-5,18) if sign==1 else (-18,5)) if head else ((36,60) if sign==1 else (-60,-36))
            def cross(k):
                idx=np.flatnonzero((x[:-1]<k)&(x[1:]>=k))
                if not len(idx):return np.nan
                n=idx[0];return t[n]+(k-x[n])*(t[n+1]-t[n])/(x[n+1]-x[n])
            t0,t1=cross(lo),cross(hi)
            if not np.isfinite(t0+t1):continue
            w.writerow([label,r['event'],'raise' if sign==1 else 'lower',round((t1-t0)*1000)])
            ax=axes[(0 if sign==1 else 1) if head else 2]
            ax.plot(t-t0,sign*x,'.-',ms=2,lw=1.2,color=color,alpha=.85)
for ax,title,unit in zip(axes,['Head raise','Head lower','Lift · both directions'],['Encoder (°)','Encoder (°)','Height (mm)']):
    ax.set(title=title,xlabel='Seconds from first crossing',ylabel=unit,xlim=(-.16,.58));ax.grid(alpha=.2)
for label,_,color in files:axes[0].plot([],[],color=color,label=label)
axes[0].legend(fontsize=8)
fig.savefig('joint-overlay.png',dpi=160)
