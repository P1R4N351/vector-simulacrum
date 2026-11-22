"""Independent pre-command axle-to-cube range from each camera and printed-fork replay."""
import argparse
import csv

import matplotlib.pyplot as plt
import numpy as np

from starting_heading import pitch, root, solve, views
from wrist_signal import params, replay


def estimate(uv,hubs,cx,f,span_locked):
    def xy(extra):
        return solve(uv,hubs,cx,f,extra,return_points=True)[:,:2]
    extra = 3.
    if span_locked:
        # Ray-plane intersections are affine in elevation. Select the lower
        # solution below the camera, on the observed front-to-rear branch.
        p0,p1 = xy(0),xy(1)
        a = p0[0]-p0[1]
        b = p1[0]-p1[1]-a
        horizontal_span = 48*np.cos(pitch)-3.75*np.sin(pitch)
        roots = np.roots([b@b,2*(a@b),a@a-horizontal_span**2])
        real = [r.real for r in roots if abs(r.imag)<1e-8 and (a+b*r.real)@a>0]
        if len(real)!=1:
            raise ValueError('Axle scale has no unique forward-facing solution')
        extra = real[0]
    front,rear = xy(extra)
    heading = (front-rear)/np.linalg.norm(front-rear)
    return -front@heading, extra


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('printed_fixture')
    args=parser.parse_args()
    v=params.values(params.load(model='vector_2'))
    clock=np.arange(-.35,.01,.001)
    def channel(file):
        a=np.loadtxt(root/file,delimiter=',',skiprows=1)
        return np.interp(clock,a[:,0],a[:,1])
    rows=[]
    fig,ax=plt.subplots(figsize=(8,3.7),layout='constrained')
    rng=np.random.default_rng(39)
    for i,(view,(face,top,hubs,cx)) in enumerate(views.items()):
        uv=np.asarray(face+top,float)
        hubs=np.asarray(hubs,float)
        fork_range=[]
        def observe(t,m,d,fork,cube):
            if t<=0:
                fork_range.append(d.xpos[cube,0]*1000)
        replay(args.printed_fixture,True,v,clock,channel('encoder.csv'),channel('dock-pitch.csv'),
               observe,yaw_deg=2 if view=='A' else -9)
        reference=fork_range[-1]
        for j,locked in enumerate((False,True)):
            n,extra=estimate(uv,hubs,cx,465,locked)
            samples=np.array([estimate(uv+rng.uniform(-2,2,uv.shape),hubs+rng.uniform(-2,2,hubs.shape),
                                       cx,rng.uniform(441.75,488.25),locked) for _ in range(300)])
            lo,hi=np.percentile(samples[:,0],[5,95])-reference
            rows.append([view,locked,extra,n,reference,n-reference,lo,hi])
            y=i*2+j
            ax.hlines(y,lo,hi,color='tab:blue')
            ax.scatter(n-reference,y,color='tab:blue')
    ax.axvline(0,color='black',ls=':')
    ax.set(yticks=range(4),yticklabels=['A · 3 mm','A · axle scale','B · 3 mm','B · axle scale'],
           xlabel='Camera range minus modeled first-touch range (mm)')
    fig.savefig(root/'seating-range.png',dpi=165)
    with (root/'seating-range.csv').open('w') as f:
        w=csv.writer(f)
        w.writerow('view axle_scale_enforced inferred_extra_axle_height_mm camera_range_mm first_touch_at_3mm_mm difference_mm sensitivity_p05_mm sensitivity_p95_mm'.split())
        w.writerows(rows)


if __name__=='__main__':
    main()
