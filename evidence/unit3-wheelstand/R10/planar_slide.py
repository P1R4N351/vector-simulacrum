"""Recover center displacement from a nominal 23 mm top glyph at both endpoints."""
from pathlib import Path
import csv
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
# Outer-border intersections in 3x enlarged crops of the corrected paired clip.
VIEWS = [
    ('A',(342,89),(1,17),[[[235,144],[350,153],[371,191],[237,177]],
                           [[313,207],[467,243],[426,290],[261,245]]]),
    ('B',(80,35),(1,18),[[[250,201],[342,203],[285,235],[188,230]],
                          [[352,160],[411,168],[349,187],[290,176]]]),
]
square = np.array([[-1,-1,0],[1,-1,0],[1,1,0],[-1,1,0]],float)*11.5
K = np.array([[279.,0.,299.5],[0.,279.,111.1],[0.,0.,1.]])


def estimate(pixels):
    ok,r,t=cv2.solvePnP(square,pixels[0],K,None,flags=cv2.SOLVEPNP_IPPE)
    assert ok
    def residual(v):
        second=Rotation.from_rotvec([0,0,v[8]]).apply(square)+[v[6],v[7],0]
        points=np.vstack([square,second])
        uv=cv2.projectPoints(points,v[:3],v[3:6],K,None)[0].reshape(2,4,2)
        return (uv-pixels).ravel()
    result=least_squares(residual,[*r.ravel(),*t.ravel(),0,0,0],max_nfev=160)
    v=result.x
    return np.hypot(*v[6:8]),(np.degrees(v[8])+45)%90-45,np.sqrt(np.mean(result.fun**2))


def main():
    cap=cv2.VideoCapture(str(HERE/'paired.mp4'))
    fig,axes=plt.subplots(2,2,figsize=(8,6),layout='constrained')
    rng=np.random.default_rng(10)
    with (HERE/'planar-slide.csv').open('w') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['view','travel_mm','yaw_mod90_deg','fit_rms_px','pick_p05_mm','pick_p95_mm'])
        for row,(view,offset,times,coords) in enumerate(VIEWS):
            pts=np.array(coords)/3+offset
            travel,yaw,error=estimate(pts)
            sensitivity=[estimate(pts+rng.normal(0,.75,pts.shape))[0] for _ in range(160)]
            result=[view,travel,yaw,error,*np.percentile(sensitivity,[5,95])]
            print(result)
            writer.writerow(result)
            for col,sec in enumerate(times):
                cap.set(cv2.CAP_PROP_POS_FRAMES,sec*12)
                ok,frame=cap.read()
                assert ok
                left=row*600
                ax=axes[row,col]
                ax.imshow(frame[:,left:left+600,::-1])
                p=pts[col][[0,1,2,3,0]]
                ax.plot(*p.T,'c.-',lw=.8)
                x,y=pts[col].mean(axis=0)
                ax.set(xlim=(x-65,x+65),ylim=(y+65,y-50),title=f'Camera {view} · {sec-2:+d} s from dock command')
                ax.axis('off')
    cap.release()
    fig.savefig(HERE/'planar-slide.png',dpi=150)


if __name__=='__main__':
    main()
