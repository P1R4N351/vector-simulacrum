"""Extract corrected native-timestamp crops for the archived free-cube impact."""
import csv
import json
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'twin-timing'))
from shelf_view import SERIALS, ShelfView

DOCK_MONOTONIC=3140.832918644
# Same rectification as paired.mp4, without the downsample to 600 px per view.
CROPS=((550,135,875,405),(150,75,475,345))


def extract(folder,out):
    out.mkdir(exist_ok=True,parents=True)
    samples=[]
    for view,serial,(left,top,right,bottom) in zip("AB",SERIALS,CROPS):
        path=folder/(serial+'.nut')
        packets=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0',
                    '-show_entries','packet=pts_time,pos,size','-of','json',str(path)]))['packets']
        with path.open('rb') as src:
            def decode(packet):
                src.seek(int(packet['pos']))
                payload=src.read(int(packet['size']))
                im=cv2.imdecode(np.frombuffer(payload,dtype='uint8'),cv2.IMREAD_COLOR)
                if im is None:
                    raise ValueError("Expected an MJPEG packet at its container offset")
                return im
            correction=ShelfView(serial,decode(packets[0]))
            for idx,p in enumerate(packets):
                t=float(p['pts_time'])-DOCK_MONOTONIC
                if not 4.05<=t<=5.55: continue
                im=correction.apply(decode(p))[top:bottom,left:right]
                filename=f'{view}-{idx}.png'
                assert cv2.imwrite(str(out/filename),im)
                samples.append([view,idx,t,filename])
    with (out/'frames.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow(['view','packet','dock_s','file'])
        w.writerows(samples)
    return samples


def video(samples,out):
    # Sixfold slow motion. Choose the nearest actual PTS on each view at each
    # playback tick and label it, so duplicate presentation frames are visible.
    start,end=4.1,5.45
    proc=subprocess.Popen(['ffmpeg','-loglevel','error','-y','-f','rawvideo',
        '-pix_fmt','bgr24','-s','650x270','-r','30','-i','-','-an','-c:v','libx264',
        '-crf','17','-pix_fmt','yuv420p','-movflags','+faststart',str(HERE/'native-contact.mp4')],stdin=subprocess.PIPE)
    pictures={name:cv2.imread(str(out/name)) for _,_,_,name in samples}
    for playback in np.arange(start,end,1/180):
        tiles=[]
        for view in 'AB':
            _,_,t,name=min((r for r in samples if r[0]==view),key=lambda r:abs(r[2]-playback))
            tile=pictures[name].copy()
            cv2.putText(tile,f'{view}  +{t:.3f} s',(8,22),cv2.FONT_HERSHEY_SIMPLEX,.58,(255,255,255),2)
            tiles.append(tile)
        proc.stdin.write(np.hstack(tiles).tobytes())
    proc.stdin.close()
    assert proc.wait()==0


def montage(samples,out):
    for view in 'AB':
        rows=[r for r in samples if r[0]==view and 4.12<r[2]<4.63]
        tiles=[]
        for _,idx,t,name in rows:
            pic=cv2.imread(str(out/name))
            cv2.putText(pic,f'{view}  +{t:.3f} s',(8,22),cv2.FONT_HERSHEY_SIMPLEX,.58,(255,255,255),2)
            tiles.append(pic)
        while len(tiles)%4: tiles.append(np.zeros_like(tiles[0]))
        cv2.imwrite(str(HERE/f'native-contact-{view}.jpg'),np.vstack([np.hstack(tiles[i:i+4])
                    for i in range(0,len(tiles),4)]),[cv2.IMWRITE_JPEG_QUALITY,94])


if __name__=='__main__':
    dest=Path(sys.argv[2])
    samples=extract(Path(sys.argv[1]),dest)
    montage(samples,dest)
    video(samples,dest)
