"""Localize the rear cube glyph in the physical stationary control, allowing occlusion."""
import csv
from pathlib import Path
import subprocess
import sys
import cv2
import matplotlib.pyplot as plt
import numpy as np
from shelf_view import ShelfView,SERIALS

HERE = Path(__file__).resolve().parent
X,Y = 612,189


def main(folder):
    ref = cv2.imread(str(HERE/'rear-glyph-reference.png'),cv2.IMREAD_GRAYSCALE)
    path = folder/(SERIALS[0]+'.nut')
    times = np.fromstring(subprocess.check_output(['ffprobe','-v','error','-show_entries','packet=pts_time',
                          '-of','csv=p=0',str(path)],text=True),sep='\n')
    cap=cv2.VideoCapture(str(path))
    rows=[]
    examples=[]
    for i,t in enumerate(times):
        ok,frame=cap.read()
        if not ok: raise ValueError('Incomplete video')
        if i%5: continue
        if i==0: view=ShelfView(SERIALS[0],frame)
        panel=view.apply(frame)
        gray=cv2.cvtColor(panel,cv2.COLOR_BGR2GRAY)
        # Include robot, cube, and dock in one search. No location prior for selection.
        scores=cv2.matchTemplate(gray,ref,cv2.TM_CCOEFF_NORMED)
        _,score,_,(x,y)=cv2.minMaxLoc(scores)
        h,w=ref.shape
        scores[max(0,y-h):y+h+1,max(0,x-w):x+w+1]=-1
        runner=cv2.minMaxLoc(scores)[1]
        accepted=score>=.85 and score-runner>=.1
        rows.append((t-times[0],x-X,y-Y,score,runner,accepted))
        if i%150==0:
            crop=panel[155:255,575:695].copy()
            cv2.putText(crop,f'{t-times[0]:.0f}s',(2,15),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,255,255),1)
            examples.append(crop)
    cap.release()
    with (HERE/'stationary-glyph.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['video_s','best_dx_px','best_dy_px','score','second_score','accepted'])
        w.writerows(rows)
    a=np.asarray(rows)
    keep=a[:,-1].astype(bool)
    print('usable / total',sum(keep),len(a),'distance percentiles',np.percentile(np.linalg.norm(a[keep,1:3],axis=1),[50,95,100]))
    fig,ax=plt.subplots(2,1,figsize=(9,4.7),sharex=True,layout='constrained')
    ax[0].plot(a[:,0],a[:,3],'.',ms=2)
    ax[0].axhline(.85,color='.5',ls='--')
    ax[0].set_ylabel('Glyph correlation')
    ax[1].scatter(a[keep,0],np.linalg.norm(a[keep,1:3],axis=1),s=5)
    ax[1].set(ylabel='Offset (pixels)',xlabel='Seconds from first A frame')
    fig.savefig(HERE/'stationary-glyph.png',dpi=150)
    while len(examples)%5: examples.append(np.zeros_like(examples[0]))
    cv2.imwrite(str(HERE/'stationary-glyph-visibility.jpg'),np.vstack([np.hstack(examples[i:i+5]) for i in range(0,len(examples),5)]))


if __name__=='__main__':
    main(Path(sys.argv[1]))
