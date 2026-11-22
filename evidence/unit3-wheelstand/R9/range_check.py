"""Check independent prox ranges against raw SDK XY and an optical-ray shelf intersection."""
from pathlib import Path
import csv
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import brentq
import sys
here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[2]))
from simulacrum import params,prox

v=params.values(params.load(model='vector_2'))
with (here/'range-check.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('run','initial_xy','shelf_ray_shift_mm','cube_rays','predicted_range_mm','recorded_range_mm','x_mm','y_mm'))
    for row in csv.DictReader((here/'pre-roll-ranging.csv').open()):
        name=row['run']
        sighting=next(r for r in csv.DictReader((here.parent/name/'roll-poses.csv').open())
                      if r['source']=='robot_observed_object' and float(r['sec'])<0)
        xyz=np.array([float(sighting['x_mm']),float(sighting['y_mm']),float(row['cube_z_mm'])-float(row['robot_z_mm'])])
        pos=np.array([float(row['robot_x_mm']),float(row['robot_y_mm']),0])
        body=Rotation.from_euler('ZY',[float(row['chassis_yaw']),-float(row['chassis_pitch'])])
        head=Rotation.from_euler('y',-float(row['head_angle']))
        camera=pos+body.apply(np.array(v['neck_joint_position_mm'])+head.apply(v['head_camera_position_mm']))
        shift=(22-camera[2])/(xyz[2]-camera[2])
        shelf=camera+(xyz-camera)*shift
        origin=pos+body.apply(v['tof_position_mm'])
        fan=body*Rotation.from_euler('y',90-v['tof_tilt_up_deg'],degrees=True)
        cube=Rotation.from_euler('z',float(sighting['yaw_rad']))
        sensor=prox.ProxSensor(v,0)
        def predict(center,orientation):
            directions=orientation.inv().apply(fan.apply(prox.ray_directions(v)))
            local=orientation.inv().apply(origin-center)
            with np.errstate(divide='ignore'):
                a=(-22-local)/directions;b=(22-local)/directions
            enter=np.minimum(a,b).max(axis=1);exit=np.maximum(a,b).min(axis=1)
            valid=(enter>=0)&(exit>=enter)
            quality,distance=sensor._signal(np.where(valid,enter,-1).tolist())
            return sum(valid),np.clip(distance,v['tof_report_min_mm'],v['tof_report_max_mm']) if quality else np.nan
        delta=next(float(r['grid_yaw_minus_sdk_deg']) for r in csv.DictReader((here/'optical-registration.csv').open())
                   if r['calibration']=='Dock and first cube' and r['view']==('Before roll' if name=='R8' else 'Next approach'))
        optical=Rotation.from_euler('z',delta,degrees=True)*cube
        direction=Rotation.from_euler('z',float(row['chassis_yaw'])).apply([1,0,0])
        start=np.r_[xyz[:2],22]
        correction=brentq(lambda s:predict(start+s*direction,optical)[1]-float(row['range_mm']),-12,12)
        for label,center,orientation in [('SDK XY',start,cube),('Camera ray to shelf',shelf,cube),
                                         ('Range fit and optical yaw',start+correction*direction,optical)]:
            count,distance=predict(center,orientation)
            w.writerow((name,label,np.linalg.norm(center[:2]-xyz[:2]),count,distance,row['range_mm'],*center[:2]))
