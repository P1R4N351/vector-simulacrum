"""Compare rectified pre-contact glyph interiors to orthographic SDK mesh faces."""
from pathlib import Path
import csv
import sys
from itertools import permutations, product
import cv2
import numpy as np
import matplotlib.pyplot as plt
from hub_registration import HERE, VIEWS, SIDE


def warp(image, corners):
    dest=np.float32([[0,0],[63,0],[63,63],[0,63]])
    h=cv2.getPerspectiveTransform(np.float32(corners),dest)
    return cv2.warpPerspective(image,h,(64,64))


def main(folder):
    templates={}
    for axis in 'xyz':
        for sign in [-1,1]:
            tag=f'{axis}{sign}'
            im=cv2.imread(str(folder/(tag+'corrected.png')),0)
            assert im is not None
            def edges(img):
                limits=[np.flatnonzero(line[65:395]>140)[[0,-1]]+65 for line in img[200:260]]
                return np.median(limits,axis=0)
            l,r=edges(im)
            t,b=edges(im.T)
            templates[tag]=warp(im,[[l,t],[r,t],[r,b],[l,b]])
    cap=cv2.VideoCapture(str(HERE/'paired.mp4'))
    cap.set(cv2.CAP_PROP_POS_FRAMES,12)
    ok,frame=cap.read()
    assert ok
    fig,axes=plt.subplots(2,4,figsize=(8,4.3),layout='constrained')
    scored={}
    with (HERE/'face-match.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['camera','face','ccw_image_quarter_turns','correlation'])
        for idx in range(6):
            i=idx%2
            name,offset,times,top=VIEWS[i]
            side=idx<2
            name+= ' side' if side else ' top'
            points=SIDE[i] if side else np.array(top[0])
            if idx>=4:
                cap.set(cv2.CAP_PROP_POS_FRAMES,times[1]*12)
                ok,frame=cap.read()
                assert ok
                points=np.array(top[1])[[3,0,1,2]]
                name+=' after'
            im=cv2.cvtColor(frame[:,600*i:600*(i+1)],cv2.COLOR_BGR2GRAY)
            image=warp(im,points/3+offset)
            scores=[]
            for tag,pic in templates.items():
                for k in range(4):
                    ref=np.rot90(pic,k)
                    # Score the printed symbol independently of border centering.
                    ref=cv2.GaussianBlur(ref,(0,0),1.5)
                    search=ref[5:-5,5:-5]
                    measured=image[12:-12,12:-12]
                    score=float(cv2.matchTemplate(search,measured,cv2.TM_CCOEFF_NORMED).max())
                    scores.append((score,tag,k))
            scores.sort(reverse=True)
            for score,tag,k in scores:
                w.writerow([name,tag,k,score])
                scored[idx,tag,k]=score
            print(name,scores[:4])
            if not side: continue
            axes[i,0].imshow(image,cmap='gray')
            axes[i,0].set_title('Camera '+name)
            for ax,(score,tag,k) in zip(axes[i,1:],scores):
                ax.imshow(np.rot90(templates[tag],k),cmap='gray')
                ax.set_title(f'{tag} · {score:.2f}')
            for ax in axes[i]: ax.axis('off')
    cap.release()
    # World coordinates use camera A's rectified top square: right, far, up.
    normals=np.array([[0,-1,0],[-1,0,0],[0,0,1],[0,0,1],[0,0,1],[0,0,1]])
    page_up=np.array([[0,0,1],[0,0,1],[0,1,0],[0,-1,0],[0,1,0],[0,-1,0]])
    poses=[]
    for perm in permutations(range(3)):
        for signs in product([-1,1],repeat=3):
            matrix=np.eye(3,dtype=int)[:,perm]*signs
            if np.linalg.det(matrix)<0: continue
            scores=[]
            faces=[]
            for idx,(n,u) in enumerate(zip(normals,page_up)):
                local=matrix.T@n
                axis=np.argmax(abs(local))
                tag='xyz'[axis]+str(local[axis])
                original_up=matrix[:,1 if axis==2 else 2]
                k=round(np.arctan2(-original_up@np.cross(u,n),original_up@u)/(np.pi/2))%4
                scores.append(scored[idx,tag,k])
                faces.append(tag)
            poses.append([np.mean(scores[:3]),*scores,*faces[:3],*matrix.ravel()])
    poses.sort(reverse=True)
    with (HERE/'rigid-faces.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['mean_fit','A_side','B_side','A_top','B_top_holdout','A_top_after','B_top_after','A_face','B_face','up_face']+
                   [f'r{i}{j}' for i in range(3) for j in range(3)])
        w.writerows(poses)
    print('rigid', [p[:10] for p in poses[:3]])
    fig.savefig(HERE/'face-match.png',dpi=150)


if __name__=='__main__':
    main(Path(sys.argv[1]))
