"""Reconstruct a rigid side profile from two hubs in the registered side-camera views."""
import csv
from pathlib import Path
import cv2
import numpy as np
import matplotlib.pyplot as plt

root=Path("../unit3-w34-return")
world=np.array([[-23,0,71.5],[23,0,71.5],[23,0,25.5],[-23,0,25.5],[0,-94,0]],float)
K=np.array([[465,0,499.5],[0,465,185.5],[0,0,1.0]])
anchors={
    "B":np.array([[855,90],[951,100],[921,218],[833,187],[668,273]],float),
    "A":np.array([[79,180],[166,162],[187,253],[104,292],[345,330]],float),
}
seated={"A":np.array([[254,308],[139,308]],float),"B":np.array([[763,243],[886,234]],float)}
hub=np.genfromtxt(root/"departure-hubs.csv",names=True,delimiter=",")
points=np.stack([np.array([hub[k+"_x"]-hub["static_x"]+930,
                           hub[k+"_y"]-hub["static_y"]+155]).T for k in ("front","rear")],axis=1)
span=np.hypot(48,3.75)
def project(pixels,R,C,plane):
    rays=R.T@np.linalg.inv(K)@np.c_[pixels,np.ones(2)].T
    pts=C[:,None]+rays*((np.sign(C[0])*plane-C[0])/rays[0])
    # Use chassis-forward (out of the dock) and up as the planar axes.
    return np.c_[-(93.205+pts[1]),pts[2]]

def measure(pts):
    delta=pts[0]-pts[1]
    pitch=np.arctan2(delta[1],delta[0])-np.arctan2(3.75,48)
    c,s=np.cos(pitch),np.sin(pitch)
    rotation=np.array([[c,-s],[s,c]])
    pad=pts.mean(axis=0)+rotation@np.array([-35.5-(-24),1-12.625])
    # Signed outward and up displacement relative to modeled uncompressed pins.
    return [np.linalg.norm(delta),np.degrees(pitch),pad[0]+59.710,pad[1]-13.671]

def main():
    rows=[]
    for view,pix in anchors.items():
        for n in (4,5):
            ok,rot,t=cv2.solvePnP(world[:n],pix[:n],K,None,flags=cv2.SOLVEPNP_SQPNP)
            assert ok
            rot,t=cv2.solvePnPRefineLM(world[:n],pix[:n],K,None,rot,t)
            R=cv2.Rodrigues(rot)[0]; C=(-R.T@t).ravel()
            # A single shared lateral plane, chosen at the W34 seated frame only.
            d0=measure(project(seated[view],R,C,0))[0]
            d1=measure(project(seated[view],R,C,1))[0]
            inferred=(span-d0)/(d1-d0)
            for label,plane in [("27",27),("30",30),("33",33),("span",inferred)]:
                def add(frame,time,pixels):
                    rows.append([view,n,label,plane,frame,time,*measure(project(pixels,R,C,plane))])
                add("W34",0.146,seated[view])
                if view=="B":
                    for time,pixels in zip(hub["t_s"],points):
                        add("W35",time,pixels)
    head=["view","anchors","choice","plane_mm","frame","time_s","hub_span_mm","pitch_deg","outward_error_mm","up_error_mm"]
    with open("geometry.csv","w") as f:
        w=csv.writer(f,lineterminator="\n");w.writerow(head);w.writerows(rows)
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout="constrained")
    with open("brackets.csv","w") as f:
        w=csv.writer(f,lineterminator="\n"); w.writerow(["anchors","choice","plane_mm","on_out_mm","off_out_mm","on_up_mm","off_up_mm"])
        for n in (4,5):
            for choice in ["27","30","33","span"]:
                rr=np.asarray([r[5:] for r in rows if r[0]=="B" and r[1]==n and r[2]==choice and r[4]=="W35"])
                plane=next(r[3] for r in rows if r[:3]==["B",n,choice])
                w.writerow([n,choice,plane,*np.interp([-.081,0],rr[:,0],rr[:,3]),*np.interp([-.081,0],rr[:,0],rr[:,4])])
                for i,ax in enumerate(axes):
                    ax.plot(rr[:,0],rr[:,3+i],ls="-" if n==5 else "--",alpha=1 if choice=="span" else .5,label=f"{n} anchors, {choice}")
    for ax,lo,hi,title in zip(axes,[-9,-5],[9,4],["Fore–aft pad offset (+ outward)","Vertical pad offset (+ up)"]):
        ax.axhspan(lo,hi,color="green",alpha=.1)
        ax.axvspan(-.081,0,color="grey",alpha=.2)
        ax.set(xlabel="Seconds from physical first off",ylabel="Pad center minus pin center (mm)",title=title)
    axes[1].legend(fontsize=7,ncol=2)
    fig.savefig("registration.png",dpi=160)


if __name__ == "__main__":
    main()
