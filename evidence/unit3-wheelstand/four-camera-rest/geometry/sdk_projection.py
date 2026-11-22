"""Recognize the visible SDK symbols and compare the browser layout on held-out faces."""
import csv
from itertools import permutations,product
import cv2
import numpy as np
from check import HERE, corners, seeds, names, world, fit, project, dlt
from sdk_glyphs import ASSETS, position, read_mesh


def main():
    gray=cv2.cvtColor(cv2.imread(str(HERE/"source.png")),cv2.COLOR_BGR2GRAY)
    observed=np.array([corners(gray,s,145) for s in seeds])
    atlas=cv2.imread(str(ASSETS/"cube.jpg"),cv2.IMREAD_GRAYSCALE)
    cs,_=cv2.findContours(np.uint8(atlas[:370,:260]<128),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    rects=[cv2.boundingRect(c) for c in cs if cv2.contourArea(c)>9000]
    dst=np.float32([[0,0],[95,0],[95,95],[0,95]])
    tri,uv=read_mesh()
    patches=[]
    for x,y,w,h in rects:
        center=np.array([x+w/2,y+h/2])/512
        mid=position(center,tri,uv)[0]
        dx=position(center+[w/4/512,0],tri,uv)[0]-mid
        dy=position(center+[0,h/4/512],tri,uv)[0]-mid
        xyz=np.array([mid-2*dx-2*dy,mid+2*dx-2*dy,mid+2*dx+2*dy,mid-2*dx+2*dy])
        pixels=np.float32([[x,y],[x+w,y],[x+w,y+h],[x,y+h]])
        patches.append((xyz,pixels))
    images=[cv2.warpPerspective(gray,cv2.getPerspectiveTransform(np.float32(q),dst),(96,96))
            for q in observed]
    orientations=[]
    for order in permutations(range(3)):
        for signs in product([-1,1], repeat=3):
            rot=np.eye(3)[list(order)]*signs
            if np.linalg.det(rot)<0: continue
            faces,match=[],[]
            for q,im in zip(world-22,images):
                xyz,pix=min(patches,key=lambda item:np.linalg.norm((item[0]@rot).mean(0)-q.mean(0)))
                xyz=xyz@rot
                ids=np.argmin(np.linalg.norm(q[:,None,:]-xyz[None,:,:],axis=2),axis=1)
                assert len(set(ids))==4
                patch=255-cv2.warpPerspective(atlas,cv2.getPerspectiveTransform(pix[ids],dst),(96,96))
                score=np.corrcoef(im[10:-10,10:-10].ravel(),patch[10:-10,10:-10].ravel())[0,1]
                faces.append(xyz[ids]+22)
                match.append((*pix[0].astype(int),score))
            orientations.append((np.mean([m[2] for m in match]),faces,match))
    orientations.sort(key=lambda entry:entry[0],reverse=True)
    print("Best two rigid orientation scores", [o[0] for o in orientations[:2]])
    with (HERE/"sdk-projection.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow(["orientation_rank","held_out","atlas_x","atlas_y","symbol_ncc","geometry","projection","rms_px"])
        for rank,(_,faces,match) in enumerate(orientations[:2],1):
            sdk=np.array(faces)
            for i,name in enumerate(names):
                for geometry,pts in [("nominal",world),("SDK",sdk)]:
                    keep=np.arange(3)!=i
                    x,y=pts[keep].reshape(-1,3),observed[keep].reshape(-1,2)
                    for camera,xy in [("general",dlt(x,y,pts[i])),("centered",project(pts[i],fit(x,y,False)))]:
                        error=np.sqrt(np.mean(np.sum((xy-observed[i])**2,axis=1)))
                        writer.writerow([rank,name,*match[i],geometry,camera,error])


if __name__=="__main__":
    main()
