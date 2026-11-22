"""Project the cube envelope from SDK poses with the frozen Vector 2 head camera."""
import csv
import itertools
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from scipy.spatial.transform import Rotation

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[2]))
from simulacrum import params

v = params.values(params.load(model='vector_2'))
width,height = v['camera_resolution_px']
focal = height/(2*np.tan(np.deg2rad(v['camera_fov_vertical_deg'])/2))
cal=json.loads((here.parents[2]/'web/camera_calibration_vector2.json').read_text())
def xyz(p):
    return np.array([p.get(k,0.) for k in 'xyz'])
def rotation(p):
    return Rotation.from_quat([p.get(k,0.) for k in ('q1','q2','q3','q0')])
def tilt(angle):
    return Rotation.from_rotvec([0,-angle,0])

fig,axes=plt.subplots(1,3,figsize=(10,3.4),layout='constrained')
with (here/'camera-reprojection.csv').open('w') as stream:
    w=csv.writer(stream,lineterminator='\n')
    w.writerow(['trial','observation_s_from_command','image_timestamp','state_minus_event_receipt_ms',
                'sdk_x','sdk_y','sdk_width','sdk_height','pred_x','pred_y','pred_width','pred_height','iou','unit_intrinsics_iou'])
    for trial,ax in zip(('W4','W8','W11'),axes):
        rows=[]
        for line in (Path(sys.argv[1])/f'p2wheel-{trial}.jsonl').read_text().splitlines():
            line=line.strip()
            while line:
                e,n=json.JSONDecoder().raw_decode(line);rows.append(e);line=line[n:].strip()
        start=next(e['t'] for e in rows if e.get('action')=='pop_a_wheelie' and e['kind']=='begin')
        event=max((e for e in rows if e['kind']=='robot_observed_object' and e['t']<=start
                   and e['data'].get('object_family')=='LIGHT_CUBE'),key=lambda e:e['t'])
        o=event['data']
        state=min((e for e in rows if e['kind']=='robot_state'
                   and e['data']['last_image_time_stamp']==o['timestamp']),key=lambda e:abs(e['t']-event['t']))
        s=state['data'];body=s['pose'];cube=o['pose']
        assert body.get('origin_id',0)==cube.get('origin_id',0)
        chassis=rotation(body)*tilt(s['pose_pitch_rad'])
        head=chassis*tilt(s['head_angle_rad'])
        eye=xyz(body)+chassis.apply(v['neck_joint_position_mm'])+head.apply(v['head_camera_position_mm'])
        optics=head*tilt(-np.deg2rad(v['head_camera_pitch_down_deg']))
        corners=np.array(list(itertools.product((-22.,22.),repeat=3)))
        a=optics.inv().apply(xyz(cube)+rotation(cube).apply(corners)-eye)
        assert np.all(a[:,0]>0)
        pixels=np.array([width,height])/2-focal*a[:,1:]/a[:,:1]
        lo,hi=pixels.min(axis=0),pixels.max(axis=0)
        b=o['img_rect'];sl=np.array([b['x_top_left'],b['y_top_left']]);size=np.array([b['width'],b['height']])
        area=np.prod(np.maximum(0,np.minimum(sl+size,hi)-np.maximum(sl,lo)))
        iou=area/(np.prod(hi-lo)+np.prod(size)-area)
        device_pixels=np.array([cal['cx_px'],cal['cy_px']])-np.array([cal['fx_px'],cal['fy_px']])*a[:,1:]/a[:,:1]
        dl,dh=device_pixels.min(axis=0),device_pixels.max(axis=0)
        intersect=np.prod(np.maximum(0,np.minimum(sl+size,dh)-np.maximum(sl,dl)))
        device_iou=intersect/(np.prod(dh-dl)+np.prod(size)-intersect)
        w.writerow([trial,event['t']-start,o['timestamp'],1000*(state['t']-event['t']),*sl,*size,*lo,*(hi-lo),iou,device_iou])
        for pos,extent,label,color in ((sl,size,'SDK','#c56024'),(lo,hi-lo,'Projected','#187587')):
            ax.add_patch(Rectangle(pos,*extent,fill=False,color=color,lw=2,label=label))
        ax.add_patch(Rectangle((0,0),width,height,fill=False,color='black'))
        ax.set(xlim=(min(-100,lo[0]-10),max(850,hi[0]+10)),ylim=(max(730,hi[1]+10),min(-50,lo[1]-10)),
               aspect='equal',title=trial,xlabel='Image x (pixels)')
        ax.legend(fontsize=8)
axes[0].set_ylabel('Image y (pixels)')
fig.savefig(here/'camera-reprojection.png',dpi=155)
