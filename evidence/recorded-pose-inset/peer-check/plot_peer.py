from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

base=Path(__file__).resolve().parent
def read(name):
    with (base/name).open() as f:
        return list(csv.DictReader(f))

fig, axes=plt.subplots(2,2,figsize=(11,7),layout="constrained")
for index,(geom,desc) in enumerate([("roller","Fixed axles"),("shifted","Shifted wheels")]):
    ax=axes[0,index]
    for fname,swap in [('crossed_summary.csv',False),('swapped_multiccd_summary.csv',True)]:
        rows=[r for r in read(fname) if r['robot']=='P2Y8' and r['implementation']==geom]
        on=(geom=='shifted')!=swap
        ax.plot([float(r['inset_mm']) for r in rows],[float(r['rms_yaw_error_pct']) for r in rows],
                marker='o',color='#166a73' if on else '#ae602c',label='Multi-contact '+('on' if on else 'off'))
    ax.set(xlabel='Inset per end (mm)',ylabel='Steady yaw RMS error (%)',title=f'P2Y8  ·  {desc}',xticks=[3.,3.75],ylim=(0,5.3))
    ax.legend()
data=read('sampling_scores.csv')
for ax,unit in zip(axes[1],['P2Y8','C7K8']):
    for i,(axis,color) in enumerate([('roll','#655589'),('pitch','#397764')]):
        vals=[]
        for k in ['recorded','instant','onepole_18.65Hz','twopole_18.65Hz']:
            rows=[r for r in data if r['unit']==unit and r['source']==k]
            vals.append(np.mean([float(r[axis]) for r in rows]))
        ax.bar(np.arange(4)+(i-.5)*.32,vals,.32,color=color,label=axis.capitalize())
    ax.set_xticks(range(4),['Robot','Raw model','1 pole','2 poles'])
    ax.set(ylabel='Mean absolute rate (rad/s)',title=f'{unit}  ·  Sensor bandwidth sensitivity',ylim=(0,.64))
    ax.legend()
fig.suptitle('Collision settings change the yaw fit. Gyro filtering changes the rocking comparison.',fontsize=15)
fig.savefig(base/'peer-comparison.png',dpi=155)
