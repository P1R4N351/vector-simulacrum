"""Use a common heading and lateral position across the short departure, then predict the held-out rear pixel."""
import csv
import cv2
import numpy as np
from scipy.optimize import least_squares
from register import K, anchors, hub, points, world
from rigid import sdk_t,sdk_p

time=hub["t_s"]
idx=np.flatnonzero(time<=0.01)
# Only frames before physical last-on determine heading and lateral location.
train=idx[time[idx]<-.081]
with open("sequence.csv","w") as file:
    w=csv.writer(file,lineterminator="\n")
    w.writerow(["anchors","face_mm","pitch_bias_deg","yaw_deg","lateral_mm","train_rms_px","held_rear_rms_px","last_on_out_lo","last_on_out_hi","first_off_out_lo","first_off_out_hi","first_off_up","last_on_lat_max","first_off_lat_max"])
    for n in (4,5):
        _,rot,tr=cv2.solvePnP(world[:n],anchors["B"][:n],K,None,flags=cv2.SOLVEPNP_SQPNP)
        rot,tr=cv2.solvePnPRefineLM(world[:n],anchors["B"][:n],K,None,rot,tr)
        R=cv2.Rodrigues(rot)[0]; C=(-R.T@tr).ravel()
        for width in (27,30,33):
            for bias in (-2,0,2):
                pitch=np.interp(time,sdk_t,sdk_p)+np.radians(bias)
                def project(i,x,yaw,out,z,features):
                    f=np.array([np.sin(yaw),-np.cos(yaw),0]); lat=np.array([np.cos(yaw),np.sin(yaw),0])
                    c,s=np.cos(pitch[i]),np.sin(pitch[i])
                    xyz=np.asarray([np.array([x,-93.205-out,z])+f*(u*c-h*s)+lat*v+[0,0,u*s+h*c] for u,v,h in features])
                    return cv2.projectPoints(xyz,rot,tr,K,None)[0][:,0],xyz
                hubs=[(0,np.sign(C[0])*width,14.5),(-48,np.sign(C[0])*width,10.75)]
                pads=[(-35.5,v,1) for v in (-9.9,9.9)]
                def residual(p):
                    return np.concatenate([(project(i,*p[:2],*p[2+2*j:4+2*j],hubs)[0]-points[i]).ravel() for j,i in enumerate(train)])
                init=np.tile([-25,7],len(train))
                fit=least_squares(residual,np.r_[0,0,init],max_nfev=100)
                x,yaw=fit.x[:2]
                errors=[]; disp=[]; up=[]; lateral=[]
                for i in idx:
                    # Front hub alone sets fore-aft and height; rear pixels withheld.
                    r=least_squares(lambda p:(project(i,x,yaw,*p,hubs[:1])[0][0]-points[i,0]),[-25,7])
                    pix,_=project(i,x,yaw,*r.x,hubs)
                    if i not in train: errors.extend(pix[1]-points[i,1])
                    _,p=project(i,x,yaw,*r.x,pads)
                    disp.append(sorted(-p[:,1]-93.205+59.710))
                    up.append(p[0,2]-13.671)
                    lateral.append(np.max(np.abs(p[:,0]-np.array([-9.85,9.85]))))
                d=np.array(disp); ts=time[idx]
                w.writerow([n,width,bias,np.degrees(yaw),x,np.mean(fit.fun**2)**.5,np.mean(np.square(errors))**.5,
                            *[np.interp(t,ts,d[:,side]) for t in (-.081,0) for side in (0,1)],
                            np.interp(0,ts,up),*[np.interp(t,ts,lateral) for t in (-.081,0)]])
