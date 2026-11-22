"""Compare the visible top of the cube with its exposed post-withdrawal appearance."""
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
video=cv2.VideoCapture(str(here/'cube-settling.mkv'))
fps=video.get(cv2.CAP_PROP_FPS)
assert fps==12
frames=[]
while True:
    ok,image=video.read()
    if not ok:
        break
    frames.append(image[:,:,0])
video.release()
frames=np.array(frames)
t=58+np.arange(len(frames))/fps
# Patch includes the far two rubber corners and the upper part of the face marking.
# Its lower edge stays above the robot that partly overlaps the near cube face.
patch=frames[:,16:31,16:56].astype(float)
def similarity(reference):
    centered=patch-patch.mean(axis=(1,2),keepdims=True)
    ref=reference-reference.mean()
    return np.sum(centered*ref,axis=(1,2))/(np.linalg.norm(centered,axis=(1,2))*np.linalg.norm(ref))
late=similarity(np.median(patch[(t>=129)&(t<130)],axis=0))
early=similarity(np.median(patch[t<59],axis=0))
with (here/'optical-settle.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('camera_s','pre_roll_similarity','exposed_final_similarity'))
    w.writerows(zip(t,early,late))
for threshold in (.90,.95,.97):
    bad=np.flatnonzero((t<129)&(late<threshold))
    print(threshold, 'last below across clip',t[bad[-1]],'last below before dock begins', t[bad[t[bad]<118][-1]])
print('range from 67 to 118',late[(67<=t)&(t<118)].min(),late[(67<=t)&(t<118)].max())
fig=plt.figure(figsize=(10,5.8),layout='constrained')
gs=fig.add_gridspec(2,5,height_ratios=[2,1])
ax=fig.add_subplot(gs[0,:])
ax.plot(t,early,label='Pre-roll reference')
ax.plot(t,late,label='Cube exposed after withdrawal')
ax.set(xlim=(59,69),ylabel='Patch correlation',xlabel='Camera time (s)',title='When the visible cube top reaches its final appearance')
ax.axhline(.95,c='.5',ls=':',lw=.8)
ax.legend()
for k,sec in enumerate([58.5,65,65.33,67,129]):
    a=fig.add_subplot(gs[1,k])
    a.imshow(frames[np.argmin(abs(t-sec))],cmap='gray',vmin=0,vmax=255,interpolation='nearest')
    a.add_patch(plt.Rectangle((15.5,15.5),40,15,fill=False,color='#ffb940',lw=1))
    a.set_title(f'{sec:g} s');a.axis('off')
fig.savefig(here/'optical-settle.png',dpi=150)

# Align against two independent motions in the previously published roll clip.
# Its first frame is 54 s into the original camera recording.
clip=cv2.VideoCapture(str(here/'roll.mp4'))
roi=[]
while True:
    ok,frame=clip.read()
    if not ok:
        break
    roi.append(cv2.cvtColor(frame[:,185:330],cv2.COLOR_BGR2GRAY).astype(float))
clip.release()
activity=np.abs(np.diff(roi,axis=0)).mean(axis=(1,2))
cam=54+np.arange(1,len(roi))/fps
state=np.genfromtxt(here/'state.csv.gz',delimiter=',',names=True)
with (here/'actions.csv').open() as f:
    zero=next(float(r['sec']) for r in csv.DictReader(f) if r['action']=='roll cube')
sdk=state['sec']-zero
for name,window,observable in [('lift',(60,60.8),state['lift']>32),
                               ('drive',(61.4,62),np.abs(state['right'])>3)]:
    onset=cam[(cam>=window[0])&(cam<window[1])&(activity>1.5)][0]
    sdk_onset=sdk[(sdk>0)&observable][0]
    print(name,'camera onset',onset,'receiver onset',sdk_onset,'offset',onset-sdk_onset)
