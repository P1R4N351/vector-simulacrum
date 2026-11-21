import csv
import sys
import itertools
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

here=Path(__file__).resolve().parent
repo=Path(sys.argv[1]).resolve()
sys.path.insert(0,str(repo/'evidence/unit3-printed-fork-contact'))
import evaluate

def subtract(box, obstacle):
    low,high=box
    a=np.maximum(low,obstacle[0]);b=np.minimum(high,obstacle[1])
    if np.any(b-a<1e-6):
        return [box]
    low,high=low.copy(),high.copy()
    parts=[]
    for axis in range(3):
        if a[axis]>low[axis]+1e-6:
            end=high.copy();end[axis]=a[axis]
            parts.append((low.copy(),end));low[axis]=a[axis]
        if b[axis]<high[axis]-1e-6:
            start=low.copy();start[axis]=b[axis]
            parts.append((start,high.copy()));high[axis]=b[axis]
    return parts

def number(v):return ' '.join(f'{x*.001:.9g}' for x in v)

def make(source,label):
    tree=ET.parse(source)
    cube=tree.find(".//body[@name='cube']")
    bounds=[]; shells=[]
    for g in cube.findall('geom'):
        p=np.fromstring(g.get('pos'),sep=' ')*1000
        s=np.fromstring(g.get('size'),sep=' ')*1000
        lo,hi=p-s,p+s
        if label in ('ledge','all','shallow','deep'):
            for v in (lo,hi):
                if abs(v[2]+12.55)<.02:v[2]=-13.9
            g.set('pos',number((hi+lo)/2));g.set('size',number((hi-lo)/2))
        bounds.append((lo,hi))
        if 'shell' in g.get('name'):shells.append(g)
    ref=shells[0].attrib.copy()
    ledge=-13.9 if label in ('all','shallow','deep','ledge') else -12.55
    for axis,sign in itertools.product((0,1),(-1,1)):
        for part in ['feet','panel']:
            if label not in (part,'all','shallow','deep'):continue
            # Add only the material visible on the front plane. Sweep its unseen depth independently.
            depth={'shallow':1.5,'deep':6.5}.get(label,3.25)
            axial=sorted([sign*22,sign*(22-depth)])
            z=(-22,ledge) if part=='feet' else (-8.81,8.81)
            for side in (-1,1):
                transverse=sorted(np.multiply(side,[9,13] if part=='feet' else [11.5,14]))
                a,b=(axial,transverse) if axis==0 else (transverse,axial)
                additions=[(np.array([a[0],b[0],z[0]]),np.array([a[1],b[1],z[1]]))]
                for obstacle in bounds: additions=[v for p in additions for v in subtract(p,obstacle)]
                for lo,hi in additions:
                    ET.SubElement(cube,'geom',dict(ref,name=f'cube_shell_lip_{len(bounds)}',pos=number((hi+lo)/2),size=number((hi-lo)/2)))
                    bounds.append((lo,hi))
    dest=here/f'{label}.xml';tree.write(dest);return dest

def job(args):
    trial,label,path=args
    sys.argv[2]=str(path)
    row,_=evaluate.run((trial,'stl','overhead','weld',0,0,0))
    return [label,*row]

if __name__=='__main__':
    from concurrent.futures import ProcessPoolExecutor
    source=Path(sys.argv[2]).resolve()
    jobs=[(i,label,make(source,label)) for label in ('original','ledge','feet','panel','all','shallow','deep') for i in [1]]
    jobs=[(trial,label,path) for _,label,path in jobs for trial in range(1,6)]
    with (here/'scores.csv').open('w') as f,ProcessPoolExecutor(4) as pool:
        writer=csv.writer(f,lineterminator='\n')
        with (repo/'evidence/unit3-printed-fork-contact/scores.csv').open() as src:writer.writerow(['profile']+next(csv.reader(src)))
        writer.writerows(pool.map(job,jobs))
