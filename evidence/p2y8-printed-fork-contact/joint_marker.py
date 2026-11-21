"""Register the raised pose using two cameras; reserve the onboard side stripe."""
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from optical_clearance import quads, marker, rows
from stripe_position import outer_edge, windows
from front_edge import here, project_pixels, front

SIDE = np.array([front[1],[-22,-11.5,-11.5]])
C = np.array([912.9463,541.1996])


def residual_segment(observed,segment):
    d=segment[1]-segment[0]
    frac=np.clip(((observed-segment[0])@d)/(d@d),0,1)
    return observed-segment[0]-frac[:,None]*d


def main():
    name,receipt,*box=windows[1]
    img=cv2.imread(str(here/"front-edge/raised.png"))
    top=outer_edge(img,box)[[0,-1]]
    gray=cv2.GaussianBlur(cv2.cvtColor(img,cv2.COLOR_BGR2GRAY).astype(float),(0,0),2)
    ys=np.arange(25,86)
    xs=560+np.argmin(np.gradient(gray,axis=1)[ys,295:405],axis=1)
    side=np.column_stack((xs,ys))
    i=np.argmin(abs(rows["sec"]-(receipt-.232)))
    t=rows["sec"][i]
    origin=np.array(box[:2])
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    with (here/"front-edge/joint.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow(["optics","relative_front_weight","overhead_rms_px","front_rms_px",
                         "reserved_side_median_px","forward_mm","left_mm","rise_mm","lowest_point_mm"])
        for (lens,k,focal),ax in zip((("See3CAM",-1.499,473.11),("pinhole",0,700)),axes):
            def rectify(q):
                d=q+[420,485]-C
                return C+d/(1+k*np.sum((d/960)**2,axis=1)[:,None])
            K=np.array([[focal,0,C[0]],[0,focal,C[1]],[0,0,1.]])
            def solve(q):
                ok,r,pos=cv2.solvePnP(marker,rectify(q),K,None)
                assert ok
                return Rotation.from_rotvec(r.ravel()),pos.ravel()
            ref,center=solve(np.median(quads[rows["sec"]<0],axis=0))
            rot,pos=solve(quads[i])
            init=np.r_[(ref.inv()*rot).as_rotvec(),ref.inv().apply(pos-center)]
            def errors(v):
                rot=Rotation.from_rotvec(v[:3]);delta=v[3:]
                camera=ref.apply(rot.apply(marker)+delta)+center
                overhead=camera[:,:2]*focal/camera[:,2,None]+C-rectify(quads[i])
                onboard=residual_segment(top,project_pixels(t,rot,delta))
                return overhead,onboard
            ax.imshow(img[:,:,::-1])
            ax.plot(*(side-origin).T,color="white",lw=3,label="Reserved edge")
            ax.plot(*(top-origin).T,color="#ff914d",lw=2,label="Fitted edge")
            for w in (0,.25,1,4):
                def objective(v):
                    oh,on=errors(v)
                    return np.r_[oh.ravel(),w*on.ravel()]
                v=least_squares(objective,init,max_nfev=600).x
                rot=Rotation.from_rotvec(v[:3]);delta=v[3:]
                line=project_pixels(t,rot,delta,SIDE)
                oh,on=errors(v)
                miss=np.median(np.linalg.norm(residual_segment(side,line),axis=1))
                writer.writerow([lens,w,np.linalg.norm(oh)/2,np.linalg.norm(on)/np.sqrt(2),
                                 miss,*delta,22.15+delta[2]-22.15*sum(abs(rot.as_matrix()[2]))])
                if w in (0,1):
                    ax.plot(*(line-origin).T,ls="--" if w==0 else "-",label=f"Side, weight {w:g}")
            ax.set(xlim=(0,img.shape[1]),ylim=(img.shape[0],0),title=lens)
    axes[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(here/"front-edge/joint.png",dpi=150)


if __name__=="__main__":
    main()
