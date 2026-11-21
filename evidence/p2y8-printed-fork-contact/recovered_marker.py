"""Recover a rounded white glyph by intersecting the lines of its outside edges."""
import cv2
import numpy as np
from pathlib import Path

folder = Path(__file__).resolve().parent / "front-edge/clock-crops"
frames = [
    ("608.4458", [(197,102),(278,134),(267,202),(167,164)]),
    ("608.6804", [(192,82),(268,126),(242,192),(152,145)]),
    ("608.9294", [(190,99),(270,142),(243,211),(156,164)]),
    ("609.1799", [(190,109),(269,149),(240,213),(155,174)]),
]


def outer_corners(image, seeds):
    gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (0,0), 1.4)
    lines = []
    for a,b in zip(seeds, np.roll(seeds,-1,axis=0)):
        tangent = (b-a)/np.linalg.norm(b-a)
        normal = np.array([-tangent[1],tangent[0]])
        mid = a + np.linspace(.19,.81,41)[:,None]*(b-a)
        offsets = np.arange(-10.,10.1,.25)
        pts = (mid[:,None,:]+offsets[None,:,None]*normal).astype("float32")
        values = cv2.remap(gray.astype("float32"),pts[:,:,0],pts[:,:,1],cv2.INTER_LINEAR)
        idx = np.argmax(np.gradient(values,axis=1),axis=1)
        vx,vy,x,y = cv2.fitLine(mid+offsets[idx,None]*normal,cv2.DIST_HUBER,0,.01,.01).ravel()
        lines.append([-vy,vx,vy*x-vx*y])
    corners = np.cross(np.roll(lines,1,axis=0),lines)
    return corners[:,:2]/corners[:,2,None]


def recovered():
    extra = []
    panels = []
    for i,(stamp, seeds) in enumerate(frames):
        img = cv2.imread(str(folder/(stamp+".png")))
        q = outer_corners(img,np.array(seeds))
        cv2.polylines(img,[np.int32(q.round())],True,(50,220,30),1,cv2.LINE_AA)
        t = float("1795287"+stamp) - (1795287608.6804-5.498106956481934)
        cv2.putText(img,f"{t:.3f} s  "+("recovered" if i%2==0 else "control"),(12,25),0,.62,(255,255,255),2)
        panels.append(img)
        extra.append((t,q+[140,300]))
    cv2.imwrite(str(folder.parent/"recovered-marker.png"),np.vstack([np.hstack(panels[:2]),np.hstack(panels[2:])]))
    return extra
