"""Compare the outer front-marker edge in pixels, retaining finite segment extent."""
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize_scalar
from front_edge import here, project_pixels, reconstruct, rows

# Rectangles include the bright stripe and dark region immediately above it.
# Exclude rounded ends, the inner emblem, and the occluding fork.
windows = [("before",3.311,260,220,310,455,235,277),
           ("raised",5.848,265,0,363,447,4,130),
           ("lowering",6.160,355,65,448,492,90,175)]


def outer_edge(image, box):
    ox,oy,x0,x1,y0,y1 = box
    gray = cv2.GaussianBlur(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY).astype(float),(0,0),2)
    roi = gray[y0-oy:y1-oy+1,x0-ox:x1-ox+1]
    lo,hi=np.percentile(roi,[2,98])
    img=np.uint8(np.clip(255*(roi-lo)/(hi-lo),0,255))
    edges=cv2.Canny(img,45,100)
    # The outer (top) boundary is dark-to-bright moving down the image.
    edges[np.gradient(roi,axis=0)<=0] = 0
    lines=cv2.HoughLines(edges,1,np.pi/720,15)
    rho,theta=lines[0,0]
    slope=-np.cos(theta)/np.sin(theta)
    offset=y0+rho/np.sin(theta)-slope*x0
    x=np.arange(x0,x1+1)
    return np.column_stack((x,slope*x+offset))


def main():
    curves={(lens,s):reconstruct(k,f,s) for lens,k,f in (("See3CAM",-1.499,473.11),("pinhole",0,700))
            for s in (False,True)}
    fig,axes=plt.subplots(1,3,figsize=(12,4))
    with (here/"front-edge/position.csv").open("w") as f:
        out=csv.writer(f,lineterminator="\n")
        out.writerow(["view","lens","shelf","measured_angle_deg","signed_line_offset_px",
                      "median_nearest_segment_px","fraction_projecting_inside_segment",
                      "vertical_only_adjustment_mm","adjusted_median_segment_px"])
        for ax,(name,t,*box) in zip(axes,windows):
            img=cv2.imread(str(here/f"front-edge/{name}.png"))
            observed=outer_edge(img,box)
            origin=np.array(box[:2])
            ax.imshow(img[:,:,::-1])
            ax.plot(*(observed-origin).T,color="#ff8c49",lw=2,label="Onboard outer edge")
            idx=np.argmin(abs(rows["sec"]-(t-.232)))
            for (lens,shelf),poses in curves.items():
                rot,delta,_=poses[idx]
                segment=project_pixels(rows["sec"][idx],rot,delta)
                direction=segment[1]-segment[0]
                unit=direction/np.linalg.norm(direction)
                foot=(observed-segment[0])@unit
                offset=(observed-segment[0])@np.array([-unit[1],unit[0]])
                near=segment[0]+np.clip(foot,0,np.linalg.norm(direction))[:,None]*unit
                dist=np.linalg.norm(near-observed,axis=1)
                angle=np.degrees(np.arctan2(*(observed[-1]-observed[0])[::-1]))
                def shift_error(dz):
                    line=project_pixels(rows["sec"][idx],rot,delta+[0,0,dz])
                    step=line[1]-line[0]
                    frac=np.clip(((observed-line[0])@step)/(step@step),0,1)
                    return np.median(np.linalg.norm(observed-line[0]-frac[:,None]*step,axis=1))
                fit=minimize_scalar(shift_error,bounds=(-25,35),method="bounded")
                out.writerow([name,lens,shelf,angle,np.median(offset),np.median(dist),
                              np.mean((foot>=0)&(foot<=np.linalg.norm(direction))),fit.x,fit.fun])
                if lens=="See3CAM":
                    ax.plot(*(segment-origin).T,color="#00c5b4" if shelf else "#b09fff",
                            ls="-" if shelf else "--",label="Shelf constraint" if shelf else "Unconstrained")
            ax.autoscale(enable=True)
            ax.set_ylim(max(ax.get_ylim()),min(ax.get_ylim()))
            ax.set_title(name);ax.set_aspect("equal")
    axes[-1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(here/"front-edge/position.png",dpi=150)


if __name__=="__main__":
    main()
