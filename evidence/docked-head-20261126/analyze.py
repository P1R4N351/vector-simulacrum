from pathlib import Path
import csv,numpy as np,matplotlib.pyplot as plt
root=Path(__file__).parent
fig,axes=plt.subplots(2,2,figsize=(10,6),layout='constrained')
def rows(serial):
 return list(csv.DictReader((root/f'{serial}.csv').open()))
def samples(records):
 return np.array([[float(r[k]) for k in ('t_s','head_deg','gyro_y_rad_s','left_mm_s','right_mm_s')] for r in records if r['event']=='state'])
commands=[r for r in rows('unit2') if r['event']=='command']
a=samples(rows('unit2'))
out=[]
for i,r in enumerate(commands[1:],1):
 start=float(r['t_s'])
 end=float(commands[i+1]['t_s']) if i+1<len(commands) else start+1
 base=a[a[:,0]<start][-1,1]
 s=a[(a[:,0]>=start)&(a[:,0]<end)]
 pre=a[(a[:,0]>=start-.22)&(a[:,0]<start)]
 stable=len(pre)>=2 and np.ptp(pre[:,1])<1 and abs(base-(25 if float(r['target_deg'])<0 else -10))<2
 col=0 if float(r['target_deg'])>0 else 1
 x=s[:,0]-start
 for j,k in enumerate([1,2]):
  axes[j,col].plot(x,s[:,k],marker='.',ls='-' if stable else '--',label=f'Repeat {(i+1)//2}'+('' if stable else ' (moving at cue)'),ms=3)
 hits=np.where(np.abs(s[:,1]-base)>=2)[0]
 if len(hits):
  h=hits[0]
  out.append([i,r['target_deg'],round(1000*x[h]) if stable else '',round(base,2),round(s[:,1].max() if col==0 else s[:,1].min(),2),round(max(abs(s[:,2])),2)])
for col,title in enumerate(['Raise from −10° to +25°','Lower from +25° to −10°']):
 axes[0,col].set_title(title);axes[0,col].legend()
 axes[1,col].set_xlabel('Time after SDK command (s)')
 axes[0,col].set_ylabel('Head encoder (degrees)');axes[1,col].set_ylabel('Head gyro Y (rad/s)')
fig.suptitle('Unit 2 on charger · repeated head movements')
fig.savefig(root/'head-overlay.png',dpi=160)
with (root/'moves.csv').open('w') as f:
 w=csv.writer(f);w.writerow(['command','target_deg','first_observed_2deg_ms','precommand_deg','extreme_deg','peak_abs_gyro_y_rad_s']);w.writerows(out)
print(out)
for serial in ['unit2','unit1','unit3']:
 a=samples(rows(serial));print(serial,len(a),round(np.median(np.diff(a[:,0]))*1000,1),np.max(abs(a[:,3:])),a[:,1].ptp() if hasattr(a[:,1],'ptp') else np.ptp(a[:,1]))
