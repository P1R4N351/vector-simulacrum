"""Project the printed collision fork into the registered W8 movie."""
import csv
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import cv2 as cv
import numpy as np
from scipy.optimize import least_squares, minimize_scalar

import replay
from early_pose import DOCK,EARLY,FORK,RIM,rot
from simulacrum.camera_spins import WORLD,calibration,intersect,project,LENS_X_MM

HERE=Path(__file__).resolve().parent
# Hand picks on the unobstructed upper edges of the near and far fork uprights.
PICKS={64:((349,335),(382,321)),66:((348,325),(384,309)),
       68:((352,313),(388,301)),70:((354,311),(389,299)),
       74:((354,311),(389,299)),76:((354,311),(389,299))}
FRAMES=(49,61,*PICKS,86)


def project_fork(v,tr,t,cam,heading,lens,verts,*,lift_time=None):
    angle=replay.mjcf.lift_angle_of_height(v,tr.obs(t if lift_time is None else lift_time,"lift_mm"))
    wrist=np.array(v["lift_base_position_mm"])+v["lift_arm_length_mm"]*np.array([np.cos(angle),0,np.sin(angle)])
    local=verts+wrist
    delta=local-np.array([LENS_X_MM,0,65.5])
    # SDK pitch is positive when the robot's front rises.
    pitch=tr.obs(t,"pitch")
    delta[:,[0,2]]=delta[:,[0,2]]@rot(pitch).T
    xyz=np.column_stack((delta[:,:2]@rot(heading).T+lens,65.5+delta[:,2]))
    return project(xyz,cam)*.5


def dock_camera(focal,corners=DOCK):
    fit=least_squares(lambda p:(project(WORLD[:4],np.r_[p,focal])-corners).ravel(),calibration()[:6])
    return np.r_[fit.x,focal]


def rim_error(focal,indices):
    camera=dock_camera(focal)
    square=np.array([[-22,22],[22,22],[22,-22],[-22,-22]],float)
    xy=intersect(RIM*2,44,camera)
    angle=np.arctan2(*(xy[1]-xy[0])[::-1])
    def residual(p):
        points=square@rot(p[2]).T+p[:2]
        return ((project(np.column_stack((points,np.full(4,44))),camera)*.5-RIM)[indices]).ravel()
    fit=least_squares(residual,[*xy.mean(axis=0),angle])
    return np.sqrt(np.mean(fit.fun**2))


def overlay(fixture,video,focal,tag,corners=DOCK):
    tr=replay.Trial("W8")
    v=replay.params.values(replay.params.load(model="vector_2"),{})
    cam=dock_camera(focal,corners)
    a,b=intersect(FORK,93,cam)
    ref_yaw=np.arctan2(b[0]-a[0],a[1]-b[1])
    lens86=intersect(np.array([[790.,683.]]),65.5,cam)[0]
    if (intersect(np.array([[732.,651.]]),65.5,cam)[0]-lens86)@[np.cos(ref_yaw),np.sin(ref_yaw)]<0:
        ref_yaw+=np.pi
    lens49=intersect(EARLY[1:]*2,65.5,cam)[0]
    pose49=np.array([tr.obs(.625,k) for k in ("x_mm","y_mm")])
    def heading(t):
        return ref_yaw+tr.obs(t,"yaw")-tr.obs(3.065,"yaw")
    axle49=lens49-rot(heading(.625))@np.array([LENS_X_MM,0])
    pieces=[np.fromstring(p.get("vertex"),sep=" ").reshape(-1,3)*1000 for p in ET.parse(fixture).findall(".//asset/mesh") if p.get("name","").startswith("lift_fork_geom_stl_")]
    assert len(pieces)==58
    verts=np.vstack(pieces)
    cap=cv.VideoCapture(video)
    tiles=[]
    with (HERE/f"fork-projection-{tag}.csv").open("w",newline="") as f:
        out=csv.writer(f,lineterminator="\n")
        out.writerow(["frame","approx_s","side","pred_x_px","pred_y_px","picked_x_px","picked_y_px","dx_px","dy_px"])
        for frame in FRAMES:
            cap.set(cv.CAP_PROP_POS_FRAMES,frame)
            ok,img=cap.read()
            assert ok
            t=cap.get(cv.CAP_PROP_POS_MSEC)*.001-5.640+3.065
            h=heading(t)
            delta=np.array([tr.obs(t,k) for k in ("x_mm","y_mm")])-pose49
            lens=axle49+rot(ref_yaw-tr.obs(3.065,"yaw"))@delta+rot(h)@np.array([LENS_X_MM,0])
            if frame==86:
                lens=lens86
            uv=project_fork(v,tr,t,cam,h,lens,verts)
            mask=np.zeros(img.shape[:2],np.uint8)
            start=0
            for piece in pieces:
                pts=uv[start:start+len(piece)]
                cv.fillConvexPoly(mask,cv.convexHull(np.rint(pts*16).astype(np.int32)),255,shift=4)
                start+=len(piece)
            before=img.copy()
            contours,_=cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
            cv.drawContours(img,contours,-1,(0,215,255),1,cv.LINE_AA)
            tips=[]
            for side in (-1,1):
                p=uv[(side*verts[:,1]>17)&(verts[:,2]>0)]
                tips.append(p[p[:,1]<p[:,1].min()+1].mean(axis=0))
            for side,tip in enumerate(sorted(tips,key=lambda p:p[0])):
                if frame in PICKS:
                    observed=np.array(PICKS[frame][side])
                    cv.drawMarker(img,tuple(observed),(200,20,190),cv.MARKER_CROSS,7,1)
                    out.writerow([frame,t,side,*tip,*observed,*(tip-observed)])
            def crop(im):
                return cv.resize(im[275:390,290:435],None,fx=2,fy=2)
            tile=np.hstack([crop(before),crop(img)])
            cv.putText(tile,f"{frame}  {t:.2f} s",(5,18),0,.48,(70,255,70),1,cv.LINE_AA)
            tiles.append(tile)
    cap.release()
    cv.imwrite(str(HERE/f"fork-projection-{tag}.jpg"),np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,len(tiles)-1,2)]+[np.hstack([tiles[-1],np.zeros_like(tiles[-1])])]))


def main(fixture,video):
    fits=[]
    for omit in (None,0,1,2,3):
        indices=[i for i in range(4) if i!=omit]
        fit=minimize_scalar(lambda foc:rim_error(foc,indices),bounds=(350,1200),method="bounded")
        fits.append(fit.x)
        print("Omitted corner",omit,"focal",fit.x,"training RMS",fit.fun)
    overlay(fixture,video,498.4,"original")
    overlay(fixture,video,fits[0],"rim")


if __name__=="__main__":
    main(*sys.argv[1:])
