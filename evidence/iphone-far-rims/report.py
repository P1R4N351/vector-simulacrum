"""Run with capture directories in order: prior near, farther left, farther right."""
import sys
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
from inputs import read,rims
from metrics import measure
from register import register
from circles import circlefit

out=Path(__file__).parent
fig,axs=plt.subplots(3,3,figsize=(13,8.4),constrained_layout=True)
rows={}
for (name,title),path,axes in zip([('near','Earlier near'),('left','Farther · left-facing'),('right','Farther · right-facing')],sys.argv[1:],axs,strict=True):
    im,depth,stack,f,pp=read(Path(path))
    ell=rims(im)
    shift,cs,ds,res=register(name,im,depth)
    raw=measure(ell,stack,f,pp); reg=measure(ell,stack,f,pp,shift)
    centers,rms=circlefit(ell,f,pp)
    ax,bx,cx=axes
    bounds=(315,639,408,69) if name=='near' else (10,305,293,79)
    overlay=im.copy()
    for e,_ in ell:
        cv.ellipse(overlay,e,(255,220,30),1,cv.LINE_AA)
    ax.imshow(cv.cvtColor(overlay,cv.COLOR_BGR2RGB))
    bx.imshow(depth,vmin=60,vmax=350,cmap='viridis')
    for u,v in zip(cs,ds):
        bx.plot([u[0],v[0]],[u[1],v[1]],color='white',lw=.6)
    bx.scatter(*cs.T,s=4,c='#32e3ff')
    bx.scatter(*ds.T,s=4,c='#fc9957')
    for e,_ in ell:
        uv=np.array(e[0])+shift
        bx.plot(*uv,'w+',ms=9)
    for plot in (ax,bx):
        plot.axis('off');plot.set_xlim(*bounds[:2]);plot.set_ylim(*bounds[2:])
    ax.set_title(title+'   |   RGB rims',fontsize=10)
    bx.set_title('Depth   |   cyan RGB edge, orange depth edge',fontsize=9)
    n=len(stack);t=np.arange(n); D=reg['observed']
    for j,(label,color) in enumerate([('Rear','#2879ac'),('Front','#af531b')]):
        cx.plot(t,D[j],'.-',lw=1,color=color,label=label+' depth')
        cx.axhline(centers[j,2],color=color,ls='--',lw=1,label=label+' RGB rim')
    cx.set_ylabel('Optical-axis depth (mm)');cx.set_xlabel('Stationary frame');cx.set_xticks([0,8,17]);cx.grid(alpha=.15)
    if name=='near':cx.legend(ncol=2,fontsize=7)
    us=np.stack([np.r_[(np.array(e[0])-pp)/f,1] for e,_ in ell])
    observed=np.nanmedian(D,axis=1)
    common=centers[:,2]+np.mean(observed-centers[:,2])
    rows[name]=dict(frames=n,full_rim_margin_px=raw['clearance'],frame_centers_rear_front=[e[0] for e,_ in ell],shift_px=shift.tolist(),edge_samples=len(cs),edge_median_abs_residual_px=res,
        registered_depth_medians_mm=observed.tolist(),circle_rgb_depths_mm=centers[:,2].tolist(),circle_rgb_span_mm=np.linalg.norm(centers[1]-centers[0]),circle_outline_rms_px=rms, simple_rgb_span_mm=raw['optical'],
        raw_span_p10_median_p90_mm=np.nanpercentile(raw['span'],[10,50,90]).tolist(),registered_span_p10_median_p90_mm=np.nanpercentile(reg['span'],[10,50,90]).tolist(),
        span_with_common_range_error_only_mm=np.linalg.norm(us[0]*common[0]-us[1]*common[1]))
fig.savefig(out/'overlays.jpg',dpi=160)
(out/'results.json').write_text(json.dumps(rows,indent=2)+'\n')
