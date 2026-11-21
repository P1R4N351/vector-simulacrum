"""Replay recorded fork poses against a free rubber-corner cube. Units in mm except in MuJoCo."""
import argparse
import json
from pathlib import Path
import cv2
import matplotlib.pyplot as plt
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
K = np.array([[537., 0, 400], [0, 537., 300], [0, 0, 1]])
SQUARE = np.array([[-1.,-1,0], [1,-1,0], [1,1,0], [-1,1,0]]) * 11.5

def start_pose(pick):
    state = pick['state']
    ok, rv, tv = cv2.solvePnP(SQUARE, np.array(pick['quad']), K, None)
    if not ok:
        raise ValueError('No initial camera pose')
    normal = cv2.Rodrigues(rv)[0][:,2]
    head = state['head']
    pitch = head - np.deg2rad(4)
    basis = np.array([[0, np.sin(pitch), np.cos(pitch)], [-1, 0, 0]])
    camera_x = -13 + 18.84*np.cos(head) + 7.96*np.sin(head)
    center = basis @ (tv.ravel() + 22*normal) + [camera_x,0]
    direction = basis @ normal
    return center, np.arctan2(direction[1],direction[0])

def rotation(yaw):
    return np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])

def replay(model, rows, pick, offset):
    center, angle = start_pose(pick)
    yaw0 = np.deg2rad(pick['state']['yaw'])
    center = rotation(yaw0) @ (center + [offset,0]) + pick['state']['xy']
    yaw = angle + yaw0
    data = mujoco.MjData(model)
    data.qpos[:] = [*(center/1000), .022, np.cos(yaw/2),0,0,np.sin(yaw/2)]
    ts = np.arange(0,rows['sec'][-1],model.opt.timestep)
    x,y,h,lift = (np.interp(ts,rows['sec'],rows[k]) for k in ('x_mm','y_mm','yaw_deg','lift_mm'))
    h = np.deg2rad(h)
    # The wrist follows the 66 mm linkage and stays level as the measured lift moves.
    reach = -41 + np.sqrt(66**2 - (lift-45)**2)
    path = np.column_stack((x+reach*np.cos(h), y+reach*np.sin(h),lift))/1000
    hit = False
    for j,pos in enumerate(path):
        data.mocap_pos[0] = pos
        data.mocap_quat[0] = [np.cos(h[j]/2),0,0,np.sin(h[j]/2)]
        mujoco.mj_step(model,data)
        for contact in data.contact:
            bodies = model.geom_bodyid[contact.geom]
            if set(bodies)=={model.body('fork').id,model.body('cube').id}:
                hit=True
    return data.qpos[:2]*1000-center,hit

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('measurements',type=Path)
    args = parser.parse_args()
    inputs = np.genfromtxt(HERE/'inputs.csv',delimiter=',',names=True)
    picks = json.loads((args.measurements/'endpoint-picks.json').read_text())
    ref = np.genfromtxt(args.measurements/'comparison.csv',names=True,delimiter=',')
    model = mujoco.MjModel.from_xml_path(str(HERE/'fixture.xml'))
    offsets = [0,-5,-10,-15,-20,-25]
    fig, axes = plt.subplots(1,5,figsize=(13,3.6),sharex=True,sharey=True)
    results=[]
    for trial,ax in enumerate(axes,1):
        pick = next(p for p in picks if p['trial']==trial and p['camera']=='on')
        observed = [ref['over_forward'][trial-1],ref['over_left'][trial-1]]
        center, normal_yaw = start_pose(pick)
        # Treat the initial good range as a central ray hitting the near face.
        ranges = inputs[(inputs['trial']==trial) & (inputs['sec']>=0) & (inputs['sec']<=.7) & (inputs['found']==1)]
        distance = np.median(ranges['prox_mm'])
        face_x = 10 + distance*np.cos(np.deg2rad(6.5))
        tof_offset = face_x + (22 - center[1]*np.sin(normal_yaw))/np.cos(normal_yaw) - center[0]
        for mode,off in enumerate([*offsets,tof_offset]):
            pred,hit = replay(model,inputs[inputs['trial']==trial],pick,off)
            results.append([trial,int(mode==len(offsets)),off,*pred,int(hit),np.linalg.norm(pred-observed)])
            if mode == len(offsets):
                ax.plot(*pred,marker='D',mfc='none',mec='k',ms=8,ls='none',label='Proximity + RGB')
            else:
                ax.plot(*pred,'o',color=plt.cm.viridis(-off/28),label=f'{off} mm')
        ax.plot(*observed,'rx',ms=10,mew=2,label='Overhead')
        ax.axhline(0,color='.85',zorder=0); ax.axvline(0,color='.85',zorder=0)
        ax.set_aspect('equal',adjustable='box');ax.set_title(f'Trial {trial}');ax.set_xlabel('Forward (mm)')
    axes[0].set_ylabel('Left (mm)')
    axes[-1].legend(loc='upper left',bbox_to_anchor=(1.02,1),title='Cube registration')
    fig.tight_layout();fig.savefig(HERE/'registration.png',dpi=160)
    np.savetxt(HERE/'replay.csv',results,delimiter=',',fmt='%.4f',header='trial,uses_proximity,registration_offset_mm,forward_mm,left_mm,fork_contact,vector_error_mm',comments='')
