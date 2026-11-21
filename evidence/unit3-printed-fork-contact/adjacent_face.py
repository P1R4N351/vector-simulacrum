"""Check overhead pose on the opposite face, observed in the same exposure."""
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
from optical_clearance import marker, rows, quads, reconstruct
from recovered_marker import folder
from joint_marker import C, residual_segment

# The visible vertical face shares the top marker's corner 3 -> corner 2 side.
# Pick the central straight part of the upper white border, avoiding rounded ends.
checks = [("608.6804",5.4981,(124,187),(181,219)),
          ("609.1799",5.9976,(122,209),(181,241))]
edge = np.array([[22,-11.5,11.5],[22,11.5,11.5]])
crop_offset = np.array([140,300])
sensor_offset = np.array([420,485])


def measure(image,a,b):
    gray=cv2.GaussianBlur(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY).astype("float32"),(0,0),1)
    ax,ay=a
    bx,by=b
    xs=np.arange(ax,bx+1)
    ys=ay+(xs-ax)*(by-ay)/(bx-ax)
    grid=np.arange(-7,7.1,.25)
    y=(ys[:,None]+grid).astype("float32")
    x=np.broadcast_to(xs[:,None],y.shape).astype("float32")
    values=cv2.remap(gray,x,y,cv2.INTER_LINEAR)
    picks=ys+grid[np.argmax(np.gradient(values,axis=1),axis=1)]
    # A few columns lock on to the parallel emblem inside the border.
    vx,vy,x0,y0=cv2.fitLine(np.column_stack((xs,picks)),cv2.DIST_HUBER,0,.01,.01).ravel()
    keep=abs(picks-(y0+(xs-x0)*vy/vx))<2
    return np.column_stack([xs[keep],picks[keep]])


def project_face(q,k,f,points):
    d=q+sensor_offset-C
    pixels=C+d/(1+k*np.sum((d/960)**2,axis=1)[:,None])
    K=np.array([[f,0,C[0]],[0,f,C[1]],[0,0,1.]])
    ok,r,t=cv2.solvePnP(marker,pixels,K,None)
    if not ok:
        raise ValueError("top glyph pose unavailable")
    ideal=cv2.projectPoints(points,r,t,K,None)[0][:,0]-C
    raw=2*ideal/(1+np.sqrt(1-4*k*np.sum((ideal/960)**2,axis=1)[:,None]))
    return C+raw-sensor_offset-crop_offset


def main():
    fig,axes=plt.subplots(1,2,figsize=(9,4))
    with (folder.parent/"adjacent-face.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["sec","optics","pose","columns","median_edge_error_px"])
        for ax,(stamp,t,a,b) in zip(axes,checks):
            image=cv2.imread(str(folder/(stamp+".png")))
            obs=measure(image,a,b)
            index=np.argmin(abs(rows["sec"]-t))
            q=quads[index]
            ax.imshow(image[:,:,::-1])
            ax.plot(*obs.T,color="yellow",label="Observed outer border")
            for name,k,focal in (("See3CAM",-1.499,473.11),("Pinhole",0,700)):
                points=np.linspace(*edge,25)
                rot,delta,_=reconstruct(k,focal,True)[index]
                reference=np.median(quads[rows["sec"]<0],axis=0)
                for mode,corners,pts in (("free",q,points),("shelf",reference,rot.apply(points)+delta)):
                    line=project_face(corners,k,focal,pts)
                    errors=np.min([np.linalg.norm(residual_segment(obs,line[j:j+2]),axis=1)
                                   for j in range(len(line)-1)],axis=0)
                    w.writerow([t,name,mode,len(obs),np.median(errors)])
                    ax.plot(*line.T,label=f"{name} {mode}",ls="-" if mode=="free" else "--")
            ax.set(xlim=(65,280),ylim=(260,70),title=f"{t:.3f} s",aspect="equal")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(folder.parent/"adjacent-face.png",dpi=150)


if __name__=="__main__":
    main()
