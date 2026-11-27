"""Solve both hub-ray depths using the SDK pitch and known rigid hub separation."""
import csv
import gzip
from pathlib import Path
import cv2
import numpy as np

# Use the existing corrected camera B dock landmarks.
from register import K, anchors, points, world, hub
with gzip.open(Path("../unit3-w34-return")/"continuation.csv.gz","rt") as f:
    sdk=list(csv.DictReader(f))
edge=next(float(r["receipt_s"]) for r in sdk if not int(r["status"]) & 4096)
sdk_t=np.asarray([float(r["receipt_s"])-edge for r in sdk])
sdk_p=np.asarray([float(r["pitch_rad"]) for r in sdk])

def solutions(R,C,pixels,angle):
    rays=(R.T@np.linalg.inv(K)@np.c_[pixels,np.ones(2)].T).T
    # Front minus rear vertical separation under observed nose-up pitch.
    dz=48*np.sin(angle)+3.75*np.cos(angle)
    # Eliminate rear depth, then solve the rigid-length quadratic in front depth.
    u=rays[0]-rays[1]*rays[0,2]/rays[1,2]
    v=rays[1]*dz/rays[1,2]
    for a in np.roots([u@u,2*u@v,v@v-(48**2+3.75**2)]):
        if abs(a.imag)>1e-7:continue
        a=a.real
        b=(a*rays[0,2]-dz)/rays[1,2]
        if min(a,b)<=0:continue
        front,rear=C+a*rays[0],C+b*rays[1]
        f=front-rear
        f[2]=0
        f/=np.linalg.norm(f)
        lateral=np.array([-f[1],f[0],0])
        if f[1]>=0:continue
        midpoint=(front+rear)/2
        yield midpoint,f,lateral

def main():
    with open("rigid.csv","w") as file:
        w=csv.writer(file,lineterminator="\n")
        w.writerow(["anchors","time_s","pitch_bias_deg","branch","yaw_deg","chassis_lateral_mm","pad_out_lo_mm","pad_out_hi_mm","pad_up_mm","lateral_max_mm"])
        for n in (4,5):
            _,rot,tr=cv2.solvePnP(world[:n],anchors["B"][:n],K,None,flags=cv2.SOLVEPNP_SQPNP)
            rot,tr=cv2.solvePnPRefineLM(world[:n],anchors["B"][:n],K,None,rot,tr)
            R=cv2.Rodrigues(rot)[0]; C=(-R.T@tr).ravel()
            for time,pixels in zip(hub["t_s"],points):
                for bias in (-2,0,2):
                    pitch=np.interp(time,sdk_t,sdk_p)+np.radians(bias)
                    for i,(mid,f,lat) in enumerate(solutions(R,C,pixels,pitch)):
                        center=mid-np.sign(C[0])*30*lat
                        forward=-11.5*np.cos(pitch)+11.625*np.sin(pitch)
                        up=-11.5*np.sin(pitch)-11.625*np.cos(pitch)
                        pad=center+f*forward+[0,0,up]
                        pads=[pad+side*9.9*lat for side in (-1,1)]
                        out=sorted(-p[1]-93.205+59.710 for p in pads)
                        dy=max(abs(p[0]-side*9.85) for p,side in zip(pads,(-1,1)))
                        w.writerow([n,time,bias,i,np.degrees(np.arctan2(f[0],-f[1])),center[0],*out,pad[2]-13.671,dy])


if __name__ == "__main__":
    main()
