"""Evaluate a candidate focal length and a candidate marker size separately."""
from pathlib import Path
import sys,json,csv
import numpy as np
here=Path(__file__).resolve().parent
base=here.parent
cal=json.loads((base.parent/'web/camera_calibration_vector2.json').read_text())
sys.path.insert(0,str(base/'p2y8-cube-contact-replay'))
import replay as sim
rows=np.genfromtxt(base/'p2y8-cube-contact-replay/inputs.csv',names=True,delimiter=',')
picks=json.loads((base/'p2y8-paired-cube-push/endpoint-picks.json').read_text())
measured=np.genfromtxt(base/'p2y8-paired-cube-push/comparison.csv',names=True,delimiter=',')
model=sim.mujoco.MjModel.from_xml_path(str(base/'p2y8-cube-contact-replay/fixture.xml'))
ratio=float(sys.argv[1])/(537*23)
original=sim.SQUARE.copy()
with (here/'contacts.csv').open('w') as f:
    out=csv.writer(f,lineterminator='\n')
    out.writerow(['assumption','trial','initial_forward_mm','camera_depth_change_mm','forward_mm','left_mm','hit','error_mm'])
    for version in ['previous','focal','marker','P2Y8']:
        for i in range(1,6):
            pick=next(p for p in picks if p['camera']=='on' and p['trial']==i)
            sim.K[0,2],sim.K[1,2]=400,300
            sim.K[0,0]=sim.K[1,1]=537
            sim.SQUARE=original
            old,_=sim.start_pose(pick)
            if version=='focal':
                sim.K[0,0]=sim.K[1,1]=537*ratio
            elif version=='P2Y8':
                sim.K[:2]=[[cal['fx_px'],0,cal['cx_px']],[0,cal['fy_px'],cal['cy_px']]]
            elif version=='marker':
                sim.SQUARE=original*ratio
            new,_=sim.start_pose(pick)
            prediction,hit=sim.replay(model,rows[rows['trial']==i],pick,0)
            delta=np.array([measured['over_forward'][i-1],measured['over_left'][i-1]])
            out.writerow([version,i,new[0],new[0]-old[0],*prediction,int(hit),np.linalg.norm(delta-prediction)])

        retreat=json.loads((here/'retreat.json').read_text())
        locations=[]
        for endpoint in retreat:
            state=endpoint['state']
            center,_=sim.start_pose(endpoint)
            locations.append(sim.rotation(np.deg2rad(state['yaw']))@center+state['xy'])
        print(version,'stationary retreat apparent travel (mm)',np.linalg.norm(locations[1]-locations[0]))

import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,5,figsize=(13,3.4))
with (here/'contacts.csv').open() as fp:
    results=list(csv.DictReader(fp))
for i,ax in enumerate(axes,1):
    for label,style,x,y in [
        ('Overhead','o-',measured['over_left'][i-1],measured['over_forward'][i-1]),
        *[(r['assumption'], 'x--',float(r['left_mm']),float(r['forward_mm']))
          for r in results if int(r['trial'])==i and r['assumption'] in ('previous','P2Y8')]]:
        ax.plot([0,x],[0,y],style,label=label)
    ax.set_title(f'Trial {i}');ax.grid(alpha=.2);ax.set_xlabel('Dock-left (mm)')
    ax.set_aspect('equal',adjustable='box')
    if i>1:
        ax.set_xlim(-4,2);ax.set_ylim(-1,8)
axes[0].set_ylabel('Forward (mm)')
axes[-1].legend(loc='upper left',bbox_to_anchor=(1,1))
fig.tight_layout();fig.savefig(here/'camera-replay.png',dpi=160)
