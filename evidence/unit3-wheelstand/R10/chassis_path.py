"""Check SDK collision trajectory against visible right hub centers in camera A."""
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from hub_registration import HERE, K, VIEWS, SIDE, TOP, FACES

# Corrected camera A preview pixels. Front axle first, rear axle second.
PICKS = np.array([
    [[370,176],[304,182]], [[373,178],[308,178]], [[375,179],[308,176]],
    [[369,180],[308,174]], [[363,183],[310,170]], [[359,184],[315,168]],
    [[352,186],[317,168]],
], dtype=float)


def main():
    frames = np.arange(75,82)
    time = frames/12-2
    _,offset,_,corners=VIEWS[0]
    uv=np.vstack([corners[0],SIDE[0]])/3+offset
    ok,r,t=cv2.solvePnP(np.vstack([TOP,FACES[0]]),uv,K,None,flags=cv2.SOLVEPNP_SQPNP)
    assert ok
    rot=cv2.Rodrigues(r)[0]
    camera=-rot.T@t.ravel()
    rays=(np.concatenate([PICKS,np.ones((*PICKS.shape[:2],1))],axis=-1)@np.linalg.inv(K).T)@rot
    hubs=camera+rays*((np.array([14.5,10.75])-camera[2])/rays[:,:,2])[:,:,None]
    axis=hubs[:,0,:2]-hubs[:,1,:2]
    heading=np.unwrap(np.arctan2(axis[:,1],axis[:,0]))
    left=np.column_stack([-np.sin(heading),np.cos(heading)])
    center=hubs[:,0,:2]+28*left
    with (HERE/'actions.csv').open() as f:
        dock=next(float(x['sec']) for x in csv.DictReader(f) if x['action']=='dock' and x['kind']=='begin')
    poses=np.genfromtxt(HERE/'poses.csv.gz',names=True,delimiter=',',dtype=None,encoding='utf8')
    poses=poses[poses['source']=='robot_state']
    sdk_xy=np.column_stack([np.interp(dock+time,poses['sec'],poses[k]) for k in ('x_mm','y_mm')])
    yaws=np.unwrap(Rotation.from_quat(np.column_stack([poses[k] for k in ('qx','qy','qz','qw')])).as_euler('ZYX')[:,0])
    sdk_yaw=np.interp(dock+time,poses['sec'],yaws)
    def body_delta(xy, angle):
        c,s=np.cos(angle),np.sin(angle)
        return (xy-xy[0])@np.array([[c,-s],[s,c]])
    observed=body_delta(center,heading[0])
    modeled=body_delta(sdk_xy,sdk_yaw[0])
    angle=np.degrees(heading-heading[0])
    sdk_angle=np.degrees(sdk_yaw-sdk_yaw[0])
    span=np.linalg.norm(axis,axis=1)
    values=np.column_stack([frames,time,observed,modeled,angle,sdk_angle,span])
    np.savetxt(HERE/'chassis-path.csv',values,delimiter=',',fmt='%.4f',comments='',
               header='frame,dock_s,opt_forward_mm,opt_left_mm,sdk_forward_mm,sdk_left_mm,opt_ccw_deg,sdk_ccw_deg,opt_axle_span_mm')
    fig,axes=plt.subplots(1,3,figsize=(11,4),layout='constrained')
    for col,(y,z,label) in enumerate([(observed[:,0],modeled[:,0],'Forward (mm)'),(observed[:,1],modeled[:,1],'Left (mm)'),(angle,sdk_angle,'Heading change (° CCW)')]):
        axes[col].plot(time,y,'o-',label='Camera A')
        axes[col].plot(time,z,'x-',label='SDK pose')
        axes[col].set(xlabel='Seconds after dock command',ylabel=label)
    state=np.genfromtxt(HERE/'state.csv.gz',names=True,delimiter=',')
    gyro=[]
    for end in time:
        stamps=np.r_[time[0],state['sec'][(state['sec']>dock+time[0])&(state['sec']<dock+end)]-dock,end]
        gyro.append(np.degrees(np.trapezoid(np.interp(stamps+dock,state['sec'],state['gz']),stamps)))
    axes[2].plot(time,gyro,':',label='Integrated head gyro Z')
    axes[2].legend(fontsize=8)
    axes[0].legend()
    fig.savefig(HERE/'chassis-path.png',dpi=150)
    cap=cv2.VideoCapture(str(HERE/'paired.mp4'))
    picks,panels=plt.subplots(2,4,figsize=(10,4),layout='constrained')
    for ix,ax in enumerate(panels.flat):
        ax.axis('off')
        if ix==len(frames):
            break
        cap.set(cv2.CAP_PROP_POS_FRAMES,int(frames[ix]))
        ok,image=cap.read()
        assert ok
        ax.imshow(image[:,:,::-1])
        ax.plot(*PICKS[ix].T,'x-',color='orange',lw=.7)
        ax.set(xlim=(265,425),ylim=(215,130),title=f'+{time[ix]:.2f} s')
    cap.release()
    picks.savefig(HERE/'chassis-picks.png',dpi=150)


if __name__=='__main__':
    main()
