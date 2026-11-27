"""Determine whether a bottom-face point can be stationary between two camera poses."""
import csv
from itertools import product
import cv2
import matplotlib.pyplot as plt
import numpy as np
from release_translation import corners, here, pose

# Points relative to the center in glyph-right, glyph-down, face-inward coordinates.
bottom=np.array([[-1,1,-1],[1,1,-1],[1,1,1],[-1,1,1]])*22.15

def closest(quad):
    # Horizontal displacement is affine in material coordinates, hence its image
    # over the continuous bottom face is the convex hull of four vectors.
    hull=cv2.convexHull(np.asarray(quad,dtype=np.float32)).reshape(-1,2)
    if len(hull)>=3 and cv2.pointPolygonTest(hull,(0.,0.),False)>=0:
        return 0.
    values=[]
    for a,b in zip(hull,np.roll(hull,-1,axis=0)):
        edge=b-a
        fraction=np.clip(-a@edge/(edge@edge),0,1) if edge@edge else 0
        values.append(np.linalg.norm(a+fraction*edge))
    return min(values)

def motion(initial, final, base):
    s,rs,_=initial
    e,re,_=final
    delta=((e-s)+(bottom@(re-rs).T))@base
    center=(e-s)@base
    start_heights=bottom@rs.T@base[:,1]
    low=np.argmax(start_heights)
    planar=delta[:,[0,2]]
    return [np.hypot(center[0],center[2]),closest(planar),min(np.linalg.norm(planar,axis=1)),
            np.linalg.norm(planar[low]),*planar[low],low]

def main():
    rows=[]
    controls=[]
    for th,focal,ref in product((80,90,100),(.95,1,1.05),("before","after")):
        group=corners[corners[:,1]==th]
        k=np.array([[465*focal,0,329.5],[0,465*focal,70.5],[0,0,1.]])
        selected=group[group[:,0]<0] if ref=="before" else group[group[:,0]>6]
        _,base,_=pose(np.median(selected[:,2:],axis=0),k)
        held=group[(group[:,0]>5.15)&(group[:,0]<5.23)]
        post=group[(group[:,0]>5.33)&(group[:,0]<5.45)]
        for branch in (0,1):
            a=pose(np.median(held[:,2:],axis=0),k,branch)
            b=pose(np.median(post[:,2:],axis=0),k,branch)
            central=motion(a,b,base)
            for period in ("early","late"):
                quiet=group[group[:,0]<0] if period=="early" else group[group[:,0]>6]
                for j in range(len(quiet)-10):
                    first=quiet[j:j+3]
                    last=quiet[j+6:j+11]
                    if abs(np.median(last[:,0])-np.median(first[:,0])-.196)>.035: continue
                    qa=pose(np.median(first[:,2:],axis=0),k,branch)
                    qb=pose(np.median(last[:,2:],axis=0),k,branch)
                    controls.append([th,focal,ref,branch,period,first[0,0],*motion(qa,qb,base)[:2]])
            individual=[motion(pose(x[2:],k,branch),pose(y[2:],k,branch),base) for x,y in product(held,post)]
            rows.append([th,focal,ref,branch,*central,min(x[1] for x in individual),max(x[1] for x in individual)])
    with (here/"material-pivot.csv").open("w") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow("threshold focal reference branch center_mm min_face_mm min_corner_mm low_corner_mm low_transverse_mm low_depth_mm low_index frame_pair_min_mm frame_pair_max_mm".split())
        writer.writerows(rows)
    with (here/"material-pivot-controls.csv").open("w") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow("threshold focal reference branch quiet_period start_s center_mm min_face_mm".split())
        writer.writerows(controls)
    fig,axes=plt.subplots(1,2,figsize=(9,3.6),sharey=True,layout="constrained")
    for col,ax in enumerate(axes):
        group=[r for r in rows if r[3]==col]
        for i,name in enumerate(("Center","Closest face point","Closest corner","Lowest corner")):
            vals=[r[4+i] for r in group]
            ax.scatter(np.full(len(vals),i),vals,s=16,alpha=.55)
            print(col,name,round(min(vals),2),round(max(vals),2))
        ax.set(xticks=range(4),xticklabels=("Center","Any point","Any corner","Lowest"),title=f"Pose branch {col+1}")
        apparent=[r[-1] for r in controls if r[3]==col]
        print(col,"quiet",len(apparent),np.percentile(apparent,[50,95,100]),
              "frame pair extremes",min(r[-2] for r in group),max(r[-1] for r in group))
        ax.vlines(1.12,0,np.percentile(apparent,95),lw=4,color=".35",label="Quiet control 95th percentile")
        ax.legend(fontsize=7)
        ax.axhline(0,color=".6",lw=.8)
    axes[0].set_ylabel("Net travel along shelf (mm)")
    fig.savefig(here/"material-pivot.png",dpi=155)

if __name__=="__main__":main()
