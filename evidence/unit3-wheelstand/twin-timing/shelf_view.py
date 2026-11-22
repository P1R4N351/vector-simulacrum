"""Default inspection view for the two opposed 1280x720 See3CAM streams."""
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
from simulacrum.camera_spins import calibration

SERIALS = ('222AD609', '3815D409')


class ShelfView:
    def __init__(self, serial, neutral_frame):
        if serial not in SERIALS or neutral_frame.shape != (720,1280,3):
            raise ValueError('Expected one of the two shelf cameras at 1280x720')
        self.serial = serial
        # Transfer the prior equidistant lens estimate to 720p for inspection.
        self.f = calibration()[-1]*2/3
        self.center = np.array([639.5,359.5])
        self.pitch = np.deg2rad(17)
        self.zoom = 465.
        self.outcenter = np.array([499.5,185.5])
        yy,xx = np.indices((440,1000))
        u,v = (xx-self.outcenter[0])/self.zoom,(yy-self.outcenter[1])/self.zoom
        y = np.cos(self.pitch)*v + np.sin(self.pitch)
        z = np.cos(self.pitch) - np.sin(self.pitch)*v
        radius = np.hypot(u,y)
        scale = self.f * np.arctan2(radius,z) / np.maximum(radius,1e-12)
        mx,my = self.center[0]+u*scale,self.center[1]+y*scale
        if serial==SERIALS[1]:
            mx,my = 1279-mx,719-my
        self.maps = cv2.convertMaps(mx.astype('float32'),my.astype('float32'),cv2.CV_16SC2)
        upright = neutral_frame if serial==SERIALS[0] else neutral_frame[::-1,::-1]
        # Bare shelf patches away from the centered cube and dock.
        patch = upright[575:615,495:555]
        neutral = np.median(patch.reshape(-1,3),axis=0)
        target = np.median(neutral)
        self.gains = np.clip(target/np.maximum(neutral,1),0.4,3.)
        table = np.arange(256)[:,None]*self.gains
        # Gentle midtone lift for the shaded work surface; fixed for the clip.
        self.lut = np.uint8(np.rint(255*np.minimum(table/255,1)**.82)).reshape(256,1,3)

    def apply(self, frame):
        if frame.shape != (720,1280,3):
            raise ValueError('Shelf correction expects a 1280x720 BGR frame')
        return cv2.LUT(cv2.remap(frame,*self.maps,cv2.INTER_LINEAR),self.lut)

    def points(self, raw):
        xy = np.asarray(raw,dtype=float)
        if self.serial==SERIALS[1]:
            xy = [1279,719]-xy
        d = xy-self.center
        r = np.linalg.norm(d,axis=1)
        a = r/self.f
        x,y = (d*(np.sin(a)/np.maximum(r,1e-12))[:,None]).T
        z = np.cos(a)
        yc = y*np.cos(self.pitch)-z*np.sin(self.pitch)
        zc = y*np.sin(self.pitch)+z*np.cos(self.pitch)
        return self.outcenter+self.zoom*np.column_stack([x/zc,yc/zc])


if __name__=='__main__':
    folder = Path(sys.argv[1])
    panels=[]
    for serial in SERIALS:
        cap=cv2.VideoCapture(str(folder/(serial+'.nut')))
        ok,ref=cap.read()
        if not ok: raise ValueError(serial)
        view=ShelfView(serial,ref)
        for _ in range(30): ok,frame=cap.read()
        panels.append(view.apply(frame))
        cap.release()
        print(serial,view.gains)
    cv2.imwrite(str(folder/'corrected.jpg'),np.hstack([cv2.resize(p,(600,264)) for p in panels]))
