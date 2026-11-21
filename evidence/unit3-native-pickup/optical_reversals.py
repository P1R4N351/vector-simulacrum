"""Track face glyph and exposed lift features in overhead crops during the reversals."""
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
# Regions in uncropped overhead pixels. Keep the cube glyph and near lift arm separate.
regions = {
    3: [[(200,315),(265,273),(313,350),(246,397)],
        [(337,243),(374,220),(398,220),(448,278),(457,303),(441,319),(416,297),(390,255),(343,269)]],
    4: [[(191,217),(248,186),(315,212),(251,258)],
        [(374,209),(400,190),(418,187),(438,202),(459,242),(462,257),(447,264),(424,230),(409,211),(383,231)]],
}
offset = np.array([75,145])
fig, axes = plt.subplots(2,3,figsize=(12,6))
rows = []
for row, (trial, polygons) in enumerate(regions.items()):
    frames = sorted((float(p.stem.split("_")[1]),p) for p in (here/"reversal-frames").glob(f"{trial}_*.png"))
    ref = min((x for x in frames if x[0]<0),key=lambda x:abs(x[0]+.25))
    gray = lambda path: cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    source = gray(ref[1])
    ax = axes[row,0]
    ax.imshow(source,cmap="gray",vmin=0,vmax=255)
    groups = []
    for polygon, color in zip(polygons,["#00d4e8","#ffae42"]):
        mask=np.zeros_like(source)
        cv2.fillPoly(mask,[np.array(polygon)-offset],255)
        points=cv2.goodFeaturesToTrack(source,60,.015,3,mask=mask)
        groups.append(points)
        ax.scatter(*points.reshape(-1,2).T,s=5,c=color)
    ax.set_title(f"Trial {trial} · cube and lift")
    ax.axis("off")
    center=np.mean(groups[0],axis=0).reshape(2)
    for t,path in frames:
        transforms=[]
        counts=[]
        for points in groups:
            dst,ok,_=cv2.calcOpticalFlowPyrLK(source,gray(path),points,None)
            back,ok2,_=cv2.calcOpticalFlowPyrLK(gray(path),source,dst,None)
            good=(ok.ravel()>0)&(ok2.ravel()>0)&(np.linalg.norm((back-points)[:,0],axis=1)<.8)
            matrix,inliers=cv2.estimateAffinePartial2D(points[good],dst[good],ransacReprojThreshold=1.25)
            assert matrix is not None and inliers.sum()>=5
            transforms.append(matrix)
            counts.append(int(inliers.sum()))
        cube,arm=transforms
        disp=(cube-arm)@np.r_[center,1]
        angle=np.degrees(np.arctan2(cube[1,0],cube[0,0])-np.arctan2(arm[1,0],arm[0,0]))
        rows.append([trial,t,*disp,angle,*counts])
    data=np.array([r for r in rows if r[0]==trial])
    for j,(col,label) in enumerate([(None,"Projected offset (px)"),(4,"Projected rotation (°)")],1):
        ax=axes[row,j]
        if col is None:
            for i,k in [(2,"Image X"),(3,"Image Y")]:
                ax.plot(data[:,1],data[:,i],".-",label=k)
            ax.legend(fontsize=8)
        else:
            ax.plot(data[:,1],data[:,col],".-")
        ax.axvspan(0,1.13,color=".9",zorder=-1)
        ax.set_title(label)
        ax.set_xlabel("Seconds from first reversal")
with (here/"optical-reversals.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["trial","s","relative_x_px","relative_y_px","relative_angle_deg","cube_inliers","arm_inliers"])
    w.writerows(rows)
fig.tight_layout()
fig.savefig(here/"optical-reversals.png",dpi=150)
