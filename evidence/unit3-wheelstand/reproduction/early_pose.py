"""Register W8's early cube and fixed backpack positions and replay the first lift raise."""
import csv
from pathlib import Path
import sys

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

import replay
from simulacrum.camera_spins import calibration, project, intersect, WORLD, LENS_X_MM

HERE = Path(__file__).resolve().parent
# Coordinates on the distributed 960x540 movie. Camera projection uses 1920x1080.
EARLY = np.array([[331.5,338.5],[419.,357.]])
FORK = np.array([[696,598],[760,580]],float)
RIM = np.array([[324,326.25],[360,334],[344.5,353],[304,343.5]])
DOCK = np.array([[810,559],[881,559],[881,627],[811,627]],float)


def rot(a):
    return np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])


def measure(tr, picks, focal=498.4, height=65.5, *, square=False):
    cam = np.r_[least_squares(lambda p:(project(WORLD[:4],np.r_[p,focal])-DOCK).ravel(),calibration()[:6]).x,focal]
    a,b = intersect(FORK,93,cam)
    forward = np.array([a[1]-b[1],b[0]-a[0]])/np.linalg.norm(b-a)
    lens = intersect(np.array([[790.,683.]]),height,cam)[0]
    crown = intersect(np.array([[732.,651.]]),height,cam)[0]
    if forward@(crown-lens)<0:
        forward = -forward
    yaw = np.arctan2(forward[1],forward[0])+tr.obs(.625,"yaw")-tr.obs(3.065,"yaw")
    cube = intersect(picks[:1]*2,44,cam)[0]
    backpack = intersect(picks[1:]*2,height,cam)[0]
    if square:
        # Corners clockwise around the image starting at the far left, at movie resolution.
        corners = RIM*2
        rim = intersect(corners,44,cam)
        local = np.array([[-22,22],[22,22],[22,-22],[-22,-22]])
        angle = np.arctan2(*(rim[1]-rim[0])[::-1])
        def predicted(p):
            xy = local@rot(p[2]).T+p[:2]
            return project(np.column_stack((xy,np.full(4,44))),cam)
        fit = least_squares(lambda p:(predicted(p)-corners).ravel(),[*rim.mean(axis=0),angle])
        cube = fit.x[:2]
        cube_sdk_yaw = fit.x[2]-yaw+tr.obs(.625,"yaw")
        # Equivalent orientations of the external square, resolved by the existing SDK sighting.
        cube_sdk_yaw += np.rint((tr.cube_yaw-cube_sdk_yaw)/(np.pi/2))*np.pi/2
        return rot(-yaw)@(cube-backpack)+[LENS_X_MM,0],cube_sdk_yaw,np.sqrt(np.mean(fit.fun**2))/2
    return rot(-yaw)@(cube-backpack)+[LENS_X_MM,0]


def main(fixture,video):
    tr = replay.Trial("W8")
    nominal = measure(tr,EARLY)
    cap=cv.VideoCapture(video)
    cap.set(cv.CAP_PROP_POS_FRAMES,49)
    ok,f=cap.read()
    assert ok
    cap.release()
    cv.polylines(f,[np.rint(RIM).astype(np.int32)],True,(205,150,20),1,cv.LINE_AA)
    for (x,y),name in zip(EARLY,("Cube","Backpack")):
        cv.drawMarker(f,(round(x),round(y)),(70,210,20),cv.MARKER_CROSS,9,1)
        cv.putText(f,name,(round(x)-18,round(y)-10),0,.34,(30,145,15),1,cv.LINE_AA)
    cv.imwrite(str(HERE/"early-pose-picks.jpg"),f[270:410,275:462])
    yaw = tr.obs(.625,"yaw")
    origin = np.array([tr.obs(.625,k) for k in ("x_mm","y_mm")])
    old = rot(-yaw)@(tr.cube_xy-origin)
    probes = [("previous",old),("early",nominal)]
    for feature in range(2):
        for axis in range(2):
            for delta in (-2,2):
                p = EARLY.copy()
                p[feature,axis] += delta
                probes.append((f"feature{feature} axis{axis} {delta:+d}px",measure(tr,p)))
    for h in (63.5,67.5):
        probes.append((f"backpack {h}mm",measure(tr,EARLY,height=h)))
    square_xy,square_yaw,square_rms = measure(tr,EARLY,square=True)
    original_yaw = tr.cube_yaw
    probes += [("square XY",square_xy),("square yaw",nominal),("square XY and yaw",square_xy)]
    print("Square fit",square_xy,"heading",np.rad2deg(square_yaw),"original",np.rad2deg(original_yaw),"pixel RMS per coordinate",square_rms)
    v = replay.params.values(replay.params.load(model="vector_2"),{"tip_supports_driven":1})
    m = replay.build(v,fixture)
    m.opt.timestep = v["sim_timestep_ms"]*.001/4
    fig,ax = plt.subplots(layout="constrained",figsize=(8,4))
    c = tr.cube
    t = c["sec"][::3]-tr.zero
    xyz = np.stack([c[k] for k in "xyz"],axis=1).reshape(-1,3,3)/8192
    rest=np.median(xyz[(t>=-.8)&(t<0)].reshape(-1,3),axis=0)
    ax.plot(t,np.linalg.norm(xyz-rest,axis=2).max(axis=1),"ko-",ms=3,label="Physical cube")
    with (HERE/"early-pose.csv").open("w",newline="") as f:
        out=csv.writer(f,lineterminator="\n")
        out.writerow(["estimate","forward_mm","left_mm","cube_yaw_deg","initial_overlap_mm","max_cube_top_xy_mm","peak_packet_deviation_g","first_contact_s","contact_geom"])
        for label,xy in probes:
            tr.cube_yaw = square_yaw if label in ("square yaw","square XY and yaw") else original_yaw
            tr.cube_xy = origin+rot(yaw)@xy
            sensor=[]
            contact=[]
            wrench=np.empty(6)
            def step(r,t):
                if not sensor:
                    sensor.append(r.sensor)
                if contact or t>1.65:
                    return
                for i,c in enumerate(r.d.contact):
                    if r.cube_id not in m.geom_bodyid[c.geom]:
                        continue
                    other=next(g for g in c.geom if m.geom_bodyid[g]!=r.cube_id)
                    if m.geom_bodyid[other]:
                        replay.mj.mj_contactForce(m,r.d,i,wrench)
                        if wrench[0]>=.01:
                            contact.extend((t,m.geom(other).name))
                            break
            trace,_,_,overlap=replay.run(m,v,tr,.625,tr.early,"servo:60",17,on_step=step)
            messages=sensor[0].messages
            ts=np.array([.625+s for s,_ in messages])
            slots=np.stack([np.frombuffer(p[2:],"<i2").reshape(3,3) for _,p in messages])/8192
            ref=np.array([0,0,1])*v["cube_accel_scale"]+np.array(v["cube_accel_bias_mg"])*.001
            mag=np.linalg.norm(slots-ref,axis=2).max(axis=1)
            win=(trace[:,0]>=1.3)&(trace[:,0]<=1.65)
            out.writerow([label,*xy,np.rad2deg(tr.cube_yaw),overlap,max(trace[win,2]),max(mag[(ts>=1.3)&(ts<=1.65)]),*(contact or ["",""])])
            if label in ("previous","early","square XY and yaw","feature0 axis0 +2px"):
                ax.plot(ts,mag,".--",ms=3,label={"previous":"Later registration","early":"Early glyph","square XY and yaw":"Four-corner fit","feature0 axis0 +2px":"Glyph shifted 2 px"}[label])
    ax.set_xlim(1.15,1.85)
    ax.set_xlabel("Seconds after wheelstand request")
    ax.set_ylabel("Peak packet deviation from rest (g)")
    ax.legend()
    fig.savefig(HERE/"early-pose.png",dpi=160)


if __name__=="__main__":
    main(*sys.argv[1:])
