"""Verify the native 800x600 renderer against unit 3's measured SDK intrinsics."""
from pathlib import Path
import csv
import json
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

out=Path(__file__).resolve().parent
web=out.parents[2]/'web'
cal=json.loads((web/'camera_calibration_vector2.json').read_text())
width,height=cal['width_px'],cal['height_px']
source=(web/'public/robot/vector2/vector.xml').read_text()
with (out/'render-optics.csv').open('w') as file:
    w=csv.writer(file,lineterminator='\n')
    w.writerow(['projection','x_over_depth','y_up_over_depth','expected_u','expected_v','render_u','render_v','error_px'])
    for mode in ('centered','calibrated'):
        root=ET.fromstring(source)
        cam=root.find(".//camera[@name='head_cam']")
        if mode=='centered':
            for k in ('sensorsize','focalpixel','principalpixel'):
                del cam.attrib[k]
            cam.set('fovy',str(cal['reported_fov_y_deg']))
        # A mocap carrier positions test targets in the actual moving camera frame.
        rig=ET.SubElement(root.find('worldbody'),'body',name='targets',mocap='true')
        points=[(0.,0.),(-.6,.35),(.6,.35),(-.6,-.35),(.6,-.35)]
        for i,(x,y) in enumerate(points):
            ET.SubElement(rig,'geom',name=f'target{i}',type='box',size='.002 .002 .00001',
                          pos=f'{x*.15} {y*.15} -.15',contype='0',conaffinity='0')
        m=mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        d=mj.MjData(m)
        d.joint("root").qpos[2]=.3  # Clear the shelf for the lower target rays.
        d.joint('head_joint').qpos[0]=.35
        mj.mj_forward(m,d)
        camera=m.camera('head_cam').id
        d.mocap_pos[0]=d.cam_xpos[camera]
        mj.mju_mat2Quat(d.mocap_quat[0],d.cam_xmat[camera])
        mj.mj_forward(m,d)
        with mj.Renderer(m,height=height,width=width) as renderer:
            visual = mj.MjvOption()
            visual.geomgroup[1] = 0
            visual.sitegroup[:] = 0
            renderer.update_scene(d,camera='head_cam',scene_option=visual)
            renderer.enable_segmentation_rendering()
            ids=renderer.render()
            for i,(x,y) in enumerate(points):
                rows,cols=np.where((ids[:,:,0]==m.geom(f'target{i}').id)&(ids[:,:,1]==mj.mjtObj.mjOBJ_GEOM))
                assert len(cols)>5,'target occluded'
                # Pixel centres in coordinates relative to top left image boundary.
                u,v=cols.mean()+.5,rows.mean()+.5
                eu,ev=cal['cx_px']+cal['fx_px']*x,cal['cy_px']-cal['fy_px']*y
                err=np.hypot(u-eu,v-ev)
                w.writerow([mode,x,y,eu,ev,u,v,err])
                if mode=='calibrated':assert err<1.
