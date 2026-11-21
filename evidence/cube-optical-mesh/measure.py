"""Measure central straight border extents on native WebGL orthographic renders."""
import csv
from pathlib import Path
import sys
import cv2
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
frames=Path(sys.argv[1])
fig,axes=plt.subplots(2,6,figsize=(13,5))
with (here/"measurements.csv").open("w") as f:
    writer=csv.writer(f,lineterminator="\n")
    writer.writerow(["face","threshold","width_mm","height_mm","center_x_mm","center_y_mm"])
    for row,mode in enumerate(("previous","corrected")):
        for col,(axis,sign) in enumerate((a,s) for a in "xyz" for s in (-1,1)):
            name=f"{axis}{sign}"
            img=cv2.imread(str(frames/(name+mode+".png")),0)
            ax=axes[row,col]
            ax.imshow(img,cmap="gray",vmin=0,vmax=255)
            if mode=="corrected":
                for threshold in (110,140,170):
                    def bounds(image):
                        # Central scanlines cross both parallel sides clear of rounded corners.
                        out=[]
                        for line in image[200:260]:
                            v=np.flatnonzero(line[65:395]>threshold)+65
                            out.append([v.min()-.5,v.max()+.5])
                        return np.median(out,axis=0)
                    x,y=bounds(img),bounds(img.T)
                    writer.writerow([name,threshold,round(np.ptp(x)/10,2),round(np.ptp(y)/10,2),
                                     round((x.mean()-229.5)/10,2),round((229.5-y.mean())/10,2)])
            ax.axis("off")
            if row==0: ax.set_title(("-" if sign<0 else "+")+axis.upper())
fig.text(.012,.71,"Original",rotation=90)
fig.text(.012,.27,"Corrected",rotation=90)
fig.tight_layout(rect=[.024,0,1,1])
fig.savefig(here/"faces.png",dpi=140)
