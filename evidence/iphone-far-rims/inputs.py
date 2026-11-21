from pathlib import Path
import cv2 as cv
import numpy as np
import json
import warnings

def read(source):
    def orient(a,method):
        return cv.resize(cv.rotate(a,cv.ROTATE_90_COUNTERCLOCKWISE),(640,480),interpolation=method)
    rgb=np.median([orient(cv.imread(str(p)),cv.INTER_LINEAR) for p in sorted(source.glob('*-color.jpg'))],axis=0).astype(np.uint8)
    frames=np.stack([orient(np.load(p),cv.INTER_NEAREST)*1000 for p in sorted(source.glob('*-depth.npy'))])
    frames[frames<=0]=np.nan
    with warnings.catch_warnings():
        warnings.simplefilter('ignore',RuntimeWarning)
        depth=np.nanmedian(frames,axis=0)
    m=json.loads((source/'metadata.json').read_text()); k=m['K'];w,h=m['originalSize']
    focal=np.array([k[4]*640/h,k[0]*480/w]); pp=np.array([k[7]*640/h,(w-1-k[6])*480/w])
    return rgb,depth,frames,focal,pp

def rims(im):
    mask=cv.inRange(cv.cvtColor(im,cv.COLOR_BGR2HSV),(115,60,60),(159,255,255))
    es=[]
    for cont in cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)[0]:
        if cv.contourArea(cont)<500:continue
        pts=cont.reshape(-1,2)
        good=pts[(pts[:,0]>1)&(pts[:,0]<638)]
        if len(good)<5:continue
        e=cv.fitEllipse(good)
        es.append((e, cv.boundingRect(cont)))
    return sorted(es,key=lambda p:max(p[0][1]))
