from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt
root=Path(__file__).parent
records=list(csv.DictReader((root/'states.csv').open()))
a=np.array([[float(r[k]) for k in ('s','head_deg','gyro_y','left','right','status')] for r in records if r['event']=='state'])
fig,axes=plt.subplots(2,2,figsize=(9,5.8),sharex=True,layout='constrained')
results=[]
for r in [r for r in records if r['event']=='trial']:
 t=float(r['s']); source=int(r['from_deg']); d=np.sign(8-source)
 pre=a[a[:,0]<t][-1]
 s=a[(a[:,0]>=t)&(a[:,0]<t+0.85)]
 row=0 if d>0 else 1
 color='teal' if abs(source-8)==12 else 'darkorange'
 label=f'{source}° to 8°'
 for col,k in enumerate((1,2)):
  ax=axes[row,col]
  ax.plot(s[:,0]-t,s[:,k],'.-',c=color,alpha=.75,label=label if label not in ax.get_legend_handles_labels()[1] else None)
  ax.set_ylabel(('Encoder (degrees)','Gyro Y (rad/s)')[col])
  ax.legend()
 def crossing(p):
  delta=d*(s[:,1]-pre[1])
  pos=np.flatnonzero(delta>=p*abs(8-pre[1]))
  if len(pos)==0:return float('nan')
  i=pos[0]; prev=s[i-1] if i else pre
  return np.interp(p*abs(8-pre[1]),[d*(prev[1]-pre[1]),delta[i]],[prev[0],s[i,0]])
 results.append((source,round(pre[1],2),np.round((crossing(.9)-crossing(.1))*1000),round(max(abs(s[:,2])),3),round(s[-1,1],2)))
for ax in axes[-1]:ax.set_xlabel('Seconds after SDK command')
fig.suptitle('Docked v1 head moves ending at 8° · two repeats per start')
fig.savefig(root/'head-range.png',dpi=155)
with (root/'moves.csv').open('w') as f:
 w=csv.writer(f);w.writerow(['start_target_deg','actual_start_deg','interpolated_10_90_ms','peak_abs_sampled_gyro_rad_s','last_deg']);w.writerows(results)
print(results)
print('states,median interval,peak wheels,dock states',len(a),np.median(np.diff(a[:,0])),abs(a[:,3:5]).max(),np.unique(a[:,5].astype(int)&4096))
