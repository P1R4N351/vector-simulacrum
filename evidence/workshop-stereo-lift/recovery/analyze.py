"""Use each settled fork post as a local template around its preceding encoder trough."""
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt

def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))
sdk=read("sdk.csv")
states=[r for r in sdk if r["event"]=="state"]
t=np.array([float(r["relative_s"]) for r in states])
height=np.array([float(r["lift_mm"]) for r in states])
flag=np.array([int(r["in_position"]) for r in states])
starts=[float(r["relative_s"]) for r in sdk if r["event"]=="command" and r["target"]=="52"]
cam=np.array([float(r["relative_s"]) for r in read("camera.csv")])
video=cv2.VideoCapture("lift.mp4")
frames=[]
while True:
    ok,im=video.read()
    if not ok:break
    frames.append(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY))
video.release()
assert len(frames)==len(cam)
def shift(ref,frame,x,y,half=12):
    patch=ref[y-10:y+11,x-half:x+half+1]
    search=frame[y-34:y+35,x-half-7:x+half+8]
    scores=cv2.matchTemplate(search,patch,cv2.TM_CCOEFF_NORMED)
    _,score,_,(u,v)=cv2.minMaxLoc(scores)
    if v in (0,48) or u in (0,14): return np.nan,score
    a,b,c=scores[v-1:v+2,u]
    return v-24+float(.5*(a-c)/(a-2*b+c)) if a-2*b+c else v-24,score
fig,axes=plt.subplots(2,3,figsize=(12,6),layout="constrained")
with open("motion.csv","w") as f, open("summary.csv","w") as s:
    w=csv.writer(f,lineterminator="\n");q=csv.writer(s,lineterminator="\n")
    w.writerow(["lower","time_s","left_down_px","right_down_px","left_corr","right_corr"])
    q.writerow(["lower","first_flag_s","trough_s","encoder_recovery_mm","left_rise_px","right_rise_px","minimum_corr","static_control_max_abs_px","tread_rise_left_px","tread_rise_right_px"])
    for n,(start,ax) in enumerate(zip(starts,axes.flat),1):
        first=np.flatnonzero((t>=start)&(flag==0))[0]
        ready=first+np.flatnonzero(flag[first:]==1)[0]
        trough=first+np.argmin(height[first:ready+1])
        # Midpoint of the first contiguous minimum plateau, restricted to the pre-flag period.
        end=trough
        while end<ready and abs(height[end+1]-height[trough])<.05:end+=1
        at=(t[trough]+t[end])/2
        nearest=lambda when:int(np.argmin(abs(cam-when)))
        j=nearest(at); k=nearest(t[ready]); ref=frames[nearest(t[ready]+.2)]
        yy=[];cc=[]
        for idx in (j,k):
            vals=[shift(ref,frames[idx],x,64) for x in (209,295)]
            yy.append(np.array([v[0] for v in vals]));cc.extend(v[1] for v in vals)
        control=[shift(ref,frames[i],x,206)[0] for i in (j,k) for x in (123,414)]
        tread=[shift(ref,frames[j],x,151)[0]-shift(ref,frames[k],x,151)[0] for x in (60,452)]
        q.writerow([n,t[ready]-start,at-start,height[ready]-height[trough],*(yy[0]-yy[1]),min(cc),np.nanmax(np.abs(control)),*tread])
        for idx in np.flatnonzero((cam>=start+.55)&(cam<=t[ready]+.3)):
            vals=[shift(ref,frames[idx],x,64) for x in (209,295)]
            w.writerow([n,cam[idx]-start,vals[0][0],vals[1][0],vals[0][1],vals[1][1]])
            for (v,c),color in zip(vals,["tab:blue","tab:orange"]):
                if c>=.85:ax.scatter(cam[idx]-start,-v,s=9,color=color)
        ax.axvline(cam[j]-start,color="gray",ls=":")
        ax.axvline(t[ready]-start,color="black",ls="--")
        ax.set(title=f"Lower {n}",xlabel="Seconds since command",ylabel="Fork rise vs settled (px)")
fig.savefig("recovery.png",dpi=160)
