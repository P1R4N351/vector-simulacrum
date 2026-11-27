"""Compare W8 camera fits to the visible outer edges of the pinned high-poly dock."""
import csv
import hashlib
from pathlib import Path
import sys

import cv2 as cv
import numpy as np

from fork_projection import DOCK, dock_camera, overlay, project
from simulacrum.dock import obj_to_charger_mm, MARKER_WALL_X_MM

HERE = Path(__file__).resolve().parent
SHA = "2ccfb20d5ed4b793862f8deea940632a2f4f34a0dfc76f8fd6083e53419b3bb0"
OFFSET = np.array([380,470])


def load(path):
    if hashlib.sha256(path.read_bytes()).hexdigest()!=SHA:
        raise ValueError("Unexpected dock OBJ")
    vertices,faces=[],[]
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            vertices.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            faces.append(np.array([int(x.split("/")[0])-1 for x in line.split()[1:]]))
    xyz=obj_to_charger_mm(np.array(vertices))
    return np.column_stack((-xyz[:,1],xyz[:,0]-MARKER_WALL_X_MM,xyz[:,2])),faces


def glyph_edges(gray):
    # Fit straight portions of the outer white border, avoiding rounded corners and fork occlusion.
    def line(image,along,lo,hi,sign):
        coords=[]
        for x in along:
            d=sign*np.diff(image[lo:hi+1,x-1:x+2].mean(axis=1))
            i=np.argmax(d)
            off=0.
            if 0<i<len(d)-1:
                l,m,r=d[i-1:i+2]
                if l-2*m+r<0:
                    off=.5*(l-r)/(l-2*m+r)
            coords.append(lo+i+.5+off)
        return np.polyfit(along,coords,1)
    top=line(gray,np.arange(443,487),86,94,1)
    bot=line(gray,np.arange(445,488),151,160,-1)
    left=line(gray.T,np.arange(102,125),425,436,1)
    right=line(gray.T,np.arange(103,146),496,505,-1)
    def corner(h,v):
        x=(v[0]*h[1]+v[1])/(1-v[0]*h[0])
        return [x,np.polyval(h,x)]
    return np.array([corner(top,left),corner(top,right),corner(bot,right),corner(bot,left)])+OFFSET


def outline(xyz,faces,focal,shape,corners):
    uv=project(xyz,dock_camera(focal,corners))-OFFSET
    coords=np.rint(uv*16).astype(np.int32)
    mask=np.zeros(shape[:2],np.uint8)
    for face in faces:
        cv.fillConvexPoly(mask,coords[face],255,shift=4)
    return mask


def main(obj,crop,fixture,video):
    xyz,faces=load(Path(obj))
    im=cv.imread(crop)
    gray=cv.cvtColor(im,cv.COLOR_BGR2GRAY).astype(float)
    # The outside of the dark rim is a rising edge into the bright shelf.
    def edge(profile,lo,hi):
        return lo+.5+np.argmax(np.diff(profile[lo:hi+1]))
    lip_x=np.arange(482,509,2)
    side_y=np.arange(205,246,4)
    lip=np.array([edge(gray[:,x-1:x+2].mean(axis=1),256,267) for x in lip_x])
    side=np.array([edge(gray[y-1:y+2].mean(axis=0),527,540) for y in side_y])
    grid=np.unique(np.r_[np.arange(380,825,5),498.4,720.9])
    edges=glyph_edges(gray)
    print("Refined native glyph corners",edges)
    results=[]
    for method,corners in (("published",DOCK),("edge fit",edges)):
        for focal in grid:
            mask=outline(xyz,faces,focal,im.shape,corners)
            bottom=np.array([np.flatnonzero(mask[:,x])[-1]+.5 for x in lip_x])
            right=np.array([np.flatnonzero(mask[y])[-1]+.5 for y in side_y])
            results.append([method,focal,np.sqrt(np.mean((bottom-lip)**2)),np.sqrt(np.mean((right-side)**2)),np.mean(bottom-lip),np.mean(right-side)])
    with (HERE/"dock-outline.csv").open("w",newline="") as out:
        w=csv.writer(out,lineterminator="\n")
        w.writerow(["glyph","focal_native_px","lip_rms_px","right_edge_rms_px","lip_bias_px","right_edge_bias_px"])
        w.writerows(results)
    for method in ("published","edge fit"):
        subset=[r for r in results if r[0]==method]
        best=min(subset,key=lambda r:r[2])
        print("lip best",best,"right best",min(subset,key=lambda r:r[3]))
        print("reference",[r for r in subset if r[1] in (498.4,720.9)])
    overlay(fixture,video,best[1],"dock-edges",corners=edges)
    panels=[]
    for focal,corners in ((None,DOCK),(498.4,DOCK),(720.9,DOCK),(best[1],edges)):
        view=im.copy()
        if focal is not None:
            mask=outline(xyz,faces,focal,im.shape,corners)
            paths,_=cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
            cv.drawContours(view,paths,-1,(0,150,255),1,cv.LINE_AA)
        for x,y in zip(np.r_[lip_x,side],np.r_[lip,side_y]):
            cv.circle(view,(round(x),round(y)),1,(120,240,0),-1)
        panel=view[60:280,370:552].copy()
        cv.putText(panel,"Recorded" if focal is None else f"f = {focal:g} px",(2,216),0,.43,(255,245,230),1,cv.LINE_AA)
        panels.append(panel)
    cv.imwrite(str(HERE/"dock-outline.jpg"),np.hstack(panels))


if __name__=="__main__":
    main(*sys.argv[1:])
