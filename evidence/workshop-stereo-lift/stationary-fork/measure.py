"""Track the lowered fork relative to a tread in each rectified view."""
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt

points = [(90, 116), (60, 151), (412, 113), (452, 151)]
video = cv2.VideoCapture("rest.mp4")
times = np.loadtxt("rest-camera.csv", delimiter=",", skiprows=1)[:, 1]
rows = []
for i, t in enumerate(times):
    ok, frame = video.read()
    assert ok
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if i == 0:
        reference = gray
        marked = frame.copy()
    shifts, scores = [], []
    for j, (x, y) in enumerate(points):
        if i == 0:
            cv2.rectangle(marked, (x-12,y-10), (x+12,y+10),
                          (40,200,255) if j % 2 == 0 else (200,160,40), 1)
        patch = reference[y-10:y+11,x-12:x+13]
        area = gray[y-18:y+19,x-20:x+21]
        corr = cv2.matchTemplate(area, patch, cv2.TM_CCOEFF_NORMED)
        _, score, _, (u,v) = cv2.minMaxLoc(corr)
        assert 0 < u < 16 and 0 < v < 16
        a,b,c = map(float, corr[v-1:v+2,u])
        shifts.append(v-8 + .5*(a-c)/(a-2*b+c))
        scores.append(score)
    rows.append([t,*shifts,*scores])
assert not video.read()[0]
video.release()
data = np.array(rows)
data[:,1:5] -= data[0,1:5].copy()
np.savetxt("motion.csv", data, delimiter=",", fmt="%.7f", comments="",
           header="time_s,lf_px,lt_px,rf_px,rt_px,lf_corr,lt_corr,rf_corr,rt_corr")
fig,(im,ax) = plt.subplots(2,1,figsize=(8,5.5),layout="constrained")
im.imshow(marked[:,:,::-1]); im.set_axis_off()
im.set_title("Lowered fork (yellow) and tread reference (blue)")
for j, eye in [(1,"Left eye"),(3,"Right eye")]:
    d = data[:,j] - data[:,j+1]
    ax.plot(times,d,label=eye,linewidth=1)
    print(eye, "95th absolute, maximum absolute (px)", np.percentile(abs(d),95),max(abs(d)))
ax.axhline(0,color="0.5",lw=.7); ax.legend()
ax.set(xlabel="Time from first stereo pair (s)",ylabel="Fork minus tread shift (px)")
fig.savefig("rest-overlay.png",dpi=150)
print("pairs, duration, minimum correlation",len(rows),times[-1],data[:,5:].min())
