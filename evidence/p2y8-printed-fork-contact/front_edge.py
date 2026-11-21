"""Compare the trial 1 front glyph stripe with independently fitted top-glyph poses."""
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from optical_clearance import reconstruct, rows
from evaluate import here, picks, states, start_pose, rotation, yaws, cal

# Native crop origins and rectangular search windows around the upper glyph stripe.
views = [("before",3.311,260,220,310,455,235,277),
         ("raised",5.848,265,0,363,447,4,130),
         ("lowering",6.160,355,65,448,492,90,175)]
front = np.array([[-22,11.5,11.5],[-22,-11.5,11.5]])
pick = next(p for p in picks if p["trial"]==1 and p["camera"]=="on")
center,_ = start_pose(pick)
a = np.deg2rad(pick["state"]["yaw"])
origin = np.r_[rotation(a) @ center + pick["state"]["xy"],22.15]
basis = Rotation.from_euler("z", yaws["degrees"][0],degrees=True)


def project(t,rot,delta):
    r = states[states["trial"]==1]
    x,y,yaw,h=[np.interp(t+1.5,r["sec"],r[k]) for k in ("x_mm","y_mm","yaw_deg","head_rad")]
    world = origin + basis.apply(rot.apply(front)+delta)
    body = Rotation.from_euler("z",-yaw,degrees=True).apply(world-[x,y,0])
    camera = [-13+18.84*np.cos(h)+7.96*np.sin(h),0,49+18.84*np.sin(h)-7.96*np.cos(h)-.52]
    pitch = h + np.deg2rad(5.16-4)
    rays = body-camera
    depth = rays @ [np.cos(pitch),0,np.sin(pitch)]
    u = cal["cx_px"]-cal["fx_px"]*rays[:,1]/depth
    v = cal["cy_px"]+cal["fy_px"]*(rays @ [np.sin(pitch),0,-np.cos(pitch)])/depth
    return np.rad2deg(np.arctan2(np.diff(v),np.diff(u)))[0]


def main():
    curves={(lens,s):reconstruct(k,f,s) for lens,k,f in (("See3CAM",-1.499,473.11),("pinhole",0,700))
            for s in (False,True)}
    fig,axes=plt.subplots(1,3,figsize=(11,3))
    with (here/"front-edge/angles.csv").open("w") as file:
        writer=csv.writer(file,lineterminator="\n")
        writer.writerow(["view","receipt_sec","overhead_sec","onboard_edge_deg","lens","shelf","projected_deg"])
        for ax,(name,t,ox,oy,x0,x1,y0,y1) in zip(axes,views):
            img=cv2.imread(str(here/f"front-edge/{name}.png"))
            gray=cv2.GaussianBlur(cv2.cvtColor(img,cv2.COLOR_BGR2GRAY).astype(float),(0,0),2)
            xs=np.arange(x0,x1+1)
            roi=gray[y0-oy:y1-oy+1,x0-ox:x1-ox+1]
            lo,hi=np.percentile(roi,[2,98])
            contrast=np.uint8(np.clip(255*(roi-lo)/(hi-lo),0,255))
            edge=cv2.Canny(contrast,45,100)
            lines=cv2.HoughLines(edge,1,np.pi/720,22)
            rho,theta=lines[0,0]
            slope=-np.cos(theta)/np.sin(theta)
            offset=y0+rho/np.sin(theta)-slope*x0
            angle=np.rad2deg(np.arctan(slope))
            ax.imshow(img[:,:,::-1]);ax.plot(xs-ox,slope*xs+offset-oy,color="#fb8545",lw=1.5)
            ax.set_title(f"{name} · {angle:.1f}°");ax.axis("off")
            idx=np.argmin(abs(rows["sec"]-(t-.232)))
            for (lens,shelf),poses in curves.items():
                rot,delta,_=poses[idx]
                writer.writerow([name,t,rows["sec"][idx],angle,lens,shelf,project(rows["sec"][idx],rot,delta)])
    fig.tight_layout()
    fig.savefig(here/"front-edge/overlay.png",dpi=150)

if __name__=="__main__":
    main()
