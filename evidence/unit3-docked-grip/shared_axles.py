"""Leave one of four hub centers out of a joint cube and robot camera registration."""
import csv

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

from starting_heading import world, views, root, pitch

objects, images, intrinsics, cameras = [], [], [], []
for i,(face,top,hubs,cx) in enumerate(views.values()):
    # B sees the opposite cube side; its local marker axes differ by half a turn.
    corners = world * ([-1,-1,1] if i else [1,1,1])
    uv = np.array(face+top+hubs,float)
    K = np.array([[465.,0,cx],[0,465,185.5],[0,0,1]])
    ok,r,t = cv2.solvePnP(corners,uv[:8],K,None,flags=cv2.SOLVEPNP_SQPNP)
    assert ok
    objects.append(corners)
    images.append(uv)
    intrinsics.append(K)
    cameras.extend([*r.ravel(),*t.ravel()])


def predict(p,i):
    x,y,z,yaw,width = p[12:]
    along = np.array([np.cos(yaw),np.sin(yaw),0])
    across = np.array([-along[1],along[0],0])
    front = np.array([x,y,z]) + (1 if i else -1)*width/2*across
    rear = front - along*(48*np.cos(pitch)-3.75*np.sin(pitch))
    rear[2] -= 48*np.sin(pitch)+3.75*np.cos(pitch)
    xyz = np.vstack([objects[i],front,rear])
    cam = p[i*6:i*6+6]
    return cv2.projectPoints(xyz,cam[:3],cam[3:],intrinsics[i],None)[0][:,0]


def main():
    cap=cv2.VideoCapture(str(root/'docked-contact.mp4'))
    ok,frame=cap.read()
    cap.release()
    assert ok
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    for i,ax in enumerate(axes):
        ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
        pts=images[i]
        ax.set(xlim=(pts[:,0].min()-35,pts[:,0].max()+35),
               ylim=(pts[:,1].max()+35,pts[:,1].min()-35),title=f'Camera {"AB"[i]}')
        ax.scatter(*pts[8:].T,s=75,facecolors='none',edgecolors='cyan',label='Picked hubs')
        ax.axis('off')
    rows=[]
    for width_prior,omit in ((w,h) for w in (None,56,60) for h in [None,(0,8),(0,9),(1,8),(1,9)]):
        masks=np.ones((2,10),bool)
        if omit:
            masks[omit]=False
        def residual(p):
            if width_prior is not None:
                p = np.r_[p,width_prior]
            return np.concatenate([(predict(p,i)-images[i])[masks[i]].ravel() for i in range(2)])
        initial=[*cameras,-55,0,18,0]
        if width_prior is None:
            initial.append(56)
        fit=least_squares(residual,initial,x_scale='jac',max_nfev=500)
        assert fit.success
        parameters = fit.x if width_prior is None else np.r_[fit.x,width_prior]
        x,y,z,yaw,width=parameters[12:]
        label='Both views'
        error=None
        if omit:
            i,k=omit
            point=predict(parameters,i)[k]
            error=np.linalg.norm(point-images[i][k])
            if width_prior in (None,56):
                name,color=('Free span','yellow') if width_prior is None else ('56 mm span','lime')
                axes[i].scatter(*point,c=color,marker='x',s=70,label=name if k==8 else None)
            label='AB'[i]+(' front' if k==8 else ' rear')
        rows.append([width_prior,label,np.sqrt(np.mean(np.sum(fit.fun.reshape(-1,2)**2,axis=1))),
                     error,-x*np.cos(yaw)-y*np.sin(yaw),z,width,np.rad2deg(yaw)])
    for ax in axes:
        ax.legend(fontsize=8)
    fig.savefig(root/'shared-axles.png',dpi=170)
    with (root/'shared-axles.csv').open('w') as f:
        w=csv.writer(f)
        w.writerow('fixed_span_mm omitted fit_rms_px held_out_error_px axial_range_mm axle_z_mm fitted_hub_face_span_mm heading_deg'.split())
        w.writerows(rows)


if __name__=='__main__':
    main()
