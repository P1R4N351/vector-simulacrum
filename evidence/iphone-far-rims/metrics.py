from inputs import *
import sys

def measure(e,z,f,pp,offset=(0,0)):
    rays=[]; ranges=[]; predicted=[]; clearance=[]
    for (ellipse,box),diam in zip(e,(18.5,26),strict=True):
        uv=np.array(ellipse[0]); ray=np.r_[(uv-pp)/f,1]
        x,y=np.rint(uv+offset).astype(int)
        rays.append(ray); predicted.append(np.mean(f)*diam/max(ellipse[1]))
        with warnings.catch_warnings():
            warnings.simplefilter('ignore',RuntimeWarning)
            ranges.append(np.nanmedian(z[:,y-4:y+5,x-4:x+5],axis=(1,2)))
        bx,by,bw,bh=box
        clearance.append(min(bx,by,640-bx-bw,480-by-bh))
    rays=np.array(rays);predicted=np.array(predicted);ranges=np.array(ranges)
    def sep(zs):
        return np.linalg.norm(zs[0,...,None]*rays[0]-zs[1,...,None]*rays[1],axis=-1)
    return dict(span=sep(ranges),optical=sep(predicted),observed=ranges,expected=predicted,clearance=clearance)

if __name__=='__main__':
    for name in sys.argv[1:]:
        im,z,s,f,pp=read(Path(name));e=rims(im)
        m=measure(e,s,f,pp)
        print(name, m['expected'],np.nanmedian(m['observed'],axis=1),m['optical'],np.nanmedian(m['span']),m['clearance'])
