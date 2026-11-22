"""Compare the rectified overhead glyph against six rendered physical face IDs."""
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
from recovered_marker import folder,frames,outer_corners

here=folder.parent.parent
atlas=cv2.imread(str(here.parent/'cube-optical-mesh/faces.png'),0)
labels=['-x','+x','-y','+y','-z','+z']
templates=[]
for x in [100,246,393,539,685,831]:
    template=atlas[2*221:2*290,2*x-69:2*x+69]
    templates.append(cv2.resize(template,(100,100),interpolation=cv2.INTER_AREA))
fig,axes=plt.subplots(2,4,figsize=(8,4),layout='constrained')
with (here/'upper-face-match.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('stamp','axis','best_quarter_turns','glyph_correlation'))
    for i,(stamp,seeds) in enumerate(frames):
        img=cv2.imread(str(folder/(stamp+'.png')))
        q=outer_corners(img,np.array(seeds)).astype('float32')
        h=cv2.getPerspectiveTransform(q,np.float32([[0,0],[99,0],[99,99],[0,99]]))
        patch=cv2.warpPerspective(cv2.cvtColor(img,cv2.COLOR_BGR2GRAY),h,(100,100))
        scores=[]
        for label,template in zip(labels,templates):
            values=[np.corrcoef(patch[18:82,18:82].ravel(),np.rot90(template,k)[18:82,18:82].ravel())[0,1]
                    for k in range(4)]
            k=int(np.argmax(values))
            scores.append((values[k],k,label))
            w.writerow((stamp,label,k,values[k]))
        best,k,label=max(scores)
        axes[0,i].imshow(patch,cmap='gray'); axes[0,i].set_title(stamp+' s')
        axes[1,i].imshow(np.rot90(templates[labels.index(label)],k),cmap='gray')
        axes[1,i].set_title(f'{label.upper()} · r={best:.2f}')
        print(stamp,sorted(scores,reverse=True)[:2])
for ax in axes.ravel(): ax.axis('off')
fig.savefig(here/'upper-face-match.png',dpi=160)
