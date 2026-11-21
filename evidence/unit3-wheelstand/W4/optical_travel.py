"""Track the overhead top glyph in its initial plane using masked normalized correlation."""
from pathlib import Path
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
# Outer white border intersections picked in the native 960x540 frame at video second 22.
q=np.float32([[229.3,413.2],[256.5,408.4],[263.0,431.0],[232.8,436.8]])
# 2 working pixels per mm; this upsamples a ~29 px wide glyph.
square=np.float32([[70,50],[116,50],[116,96],[70,96]])
H=cv2.getPerspectiveTransform(q,square)
vid=cv2.VideoCapture(str(here/'overhead.mp4'))
def rectify(frame):
    return cv2.warpPerspective(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),H,(170,150))
vid.set(cv2.CAP_PROP_POS_MSEC,22000)
ok, ref=vid.read(); assert ok
roi=rectify(ref)[43:104,63:124].astype(np.float32)
# Keep the glyph and a narrow strip around it, excluding the corner lights.
mask=np.zeros_like(roi,np.uint8)
mask[4:57,4:57]=255
angles=np.arange(-18,19,1)
templates=[]
for a in angles:
    M=cv2.getRotationMatrix2D((30,30),a,1)
    templates.append((cv2.warpAffine(roi,M,(61,61)),cv2.warpAffine(mask,M,(61,61), flags=cv2.INTER_NEAREST)))
rows=[];examples=[]
for idx in range(95,301):
    vid.set(cv2.CAP_PROP_POS_FRAMES,idx)
    ok, frame=vid.read()
    if not ok:break
    img=rectify(frame).astype(np.float32)
    best=(-2,None,None)
    for a,(temp,m) in zip(angles,templates):
        scores=cv2.matchTemplate(img,temp,cv2.TM_CCOEFF_NORMED,mask=m)
        scores[~np.isfinite(scores)]=-2
        _,score,_,loc=cv2.minMaxLoc(scores)
        if score>best[0]:best=(score,a,loc)
    score,a,(x,y)=best
    t=idx/5-1-22.330433
    dx,dy=(x-63)/2,(y-43)/2
    rows.append((t,dx,dy,np.hypot(dx,dy),a,score))
    if idx in (110,140,155,255):
        rot=cv2.getRotationMatrix2D((93,73),a,1)
        predicted=cv2.transform(square[None],rot)[0]+[2*dx,2*dy]
        corners=cv2.perspectiveTransform(predicted.astype(np.float32)[None],np.linalg.inv(H))[0]
        cv2.polylines(frame,[np.int32(np.round(corners))],True,(0,200,245),1)
        examples.append(frame[390:494,188:308,::-1])
r=np.array(rows)
with (here/'optical-travel.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n');w.writerow(['s_after_command','dx_mm','dy_mm','travel_mm','yaw_deg','correlation']);w.writerows(np.round(r,4))
fig,axes=plt.subplots(2,1,figsize=(8,5),sharex=True,layout='constrained')
for ax,j,label in zip(axes,(3,4),('Top glyph travel (mm)','Yaw (° CCW)')):
    sc=ax.scatter(r[:,0],r[:,j],c=r[:,5],cmap='viridis',vmin=.5,vmax=1,s=10)
    ax.axvline(4.30,color='.4',ls=':',lw=1)
    ax.set_ylabel(label)
axes[-1].set_xlabel('Seconds from wheelstand command');fig.colorbar(sc,ax=axes,label='Template correlation')
fig.savefig(here/'optical-travel.png',dpi=150)
plt.imsave(here/'optical-checks.jpg',np.concatenate(examples,axis=1))
for lo,hi in [(-4,3),(5,8),(10,16),(24,32)]:
    s=r[(r[:,0]>=lo)&(r[:,0]<hi)&(r[:,5]>.7)]
    print(lo,hi,len(s),np.median(s,axis=0) if len(s) else None)

# Independent hand picks on 3x enlarged native crops (same image origin for all).
manual=np.float32([[[150,85],[238,71],[256,142],[161,159]],
                   [[118,85],[201,76],[214,148],[118,158]],
                   [[105,87],[192,80],[199,153],[106,159]]])
rect=cv2.getPerspectiveTransform(manual[0],np.float32([[0,0],[23,0],[23,23],[0,23]]))
for second,quad in zip([22,30,52],manual):
    points=cv2.perspectiveTransform(quad[None],rect)[0]
    edges=np.roll(points,-1,axis=0)-points
    yaw=-np.angle((edges[:,0]+1j*edges[:,1])/[1,1j,-1,-1j],deg=True).mean()
    print('manual',second,'video s:',np.linalg.norm(points.mean(axis=0)-11.5),'mm',yaw,'degrees')
