"""Locate the rear cube's green pulses on early frames, then check the other view."""
import csv
from pathlib import Path
import subprocess
import sys
import cv2
import matplotlib.pyplot as plt
import numpy as np
from shelf_view import ShelfView,SERIALS

HERE=Path(__file__).resolve().parent


def video(path,origin):
    t=np.fromstring(subprocess.check_output(['ffprobe','-v','error','-show_entries','packet=pts_time',
                        '-of','csv=p=0',str(path)],text=True),sep='\n')-origin
    cap=cv2.VideoCapture(str(path))
    sig=[]
    for i in range(len(t)):
        ok,img=cap.read()
        if not ok: raise RuntimeError('Incomplete camera file')
        if not i: view=ShelfView(path.stem,img)
        small=cv2.resize(view.apply(img),(250,110),interpolation=cv2.INTER_AREA)
        b,g,r=cv2.split(small.astype('float32'))
        sig.append(cv2.blur(g-(r+b)/2,(3,3)))
        if not i: ref=small
    cap.release()
    return t,np.array(sig),ref


def main(folder):
    with (folder/'receipts.csv').open() as f:
        start=next(float(r['sec']) for r in csv.DictReader(f) if r['kind']=='start')
    fig,axes=plt.subplots(2,2,figsize=(10,5),layout='constrained')
    with (HERE/'rear-blink.csv').open('w') as out:
        w=csv.writer(out,lineterminator='\n')
        w.writerow(['camera','t_s','green_contrast','x_corrected_px','y_corrected_px'])
        for i,(cam,serial) in enumerate(zip('AB',SERIALS)):
            t,s,ref=video(folder/(serial+'.nut'),start)
            train=(t>.15)&(t<2.5)
            if i==0:
                # A shows the cube. Limit first-view identification to its silhouette.
                patch=s[train,40:61,149:168]
                amplitude=np.percentile(patch,90,axis=0)-np.percentile(patch,10,axis=0)
                y,x=np.unravel_index(np.argmax(amplitude),amplitude.shape)
                y+=40
                x+=149
            else:
                a_interp=np.interp(t,at,signal_a)
                drive=a_interp[train]-a_interp[train].mean()
                # Highest positive on-minus-off contrast anywhere in B, without
                # presuming either direct visibility or a location in this view.
                effect=np.tensordot(drive,s[train],axes=1)/np.dot(drive,drive)
                y,x=np.unravel_index(np.argmax(effect),effect.shape)
            signal=s[:,y,x]
            if i==0:
                at,signal_a=t,signal
            else:
                for label,m in [('train',train),('test',(t>=2.5)&(t<7.75)&(t<at[-1]))]:
                    print(label,sum(m),np.corrcoef(signal[m],a_interp[m])[0,1])
            print(cam,4*x+2,4*y+2,'dynamic range',np.diff(np.percentile(signal[train],[10,90]))[0])
            w.writerows((cam,ti,v,4*x+2,4*y+2) for ti,v in zip(t,signal))
            axes[i,0].imshow(ref[:,:,::-1])
            axes[i,0].plot(x,y,'y+',ms=12)
            axes[i,0].axis('off')
            axes[i,0].set_title(f'Camera {cam} · selected patch')
            axes[i,1].plot(t,signal,'.-',ms=2,lw=.8)
            axes[i,1].axvline(2.5,color='.5',ls='--')
            axes[i,1].set(xlabel='Time after blink command (s)',ylabel='Green contrast')
    fig.savefig(HERE/'rear-blink.png',dpi=150)


if __name__=='__main__':
    main(Path(sys.argv[1]))
