from analysis import *
from score import row_edges
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib as mpl
mpl.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
sky='#19738a'; amber='#d56c24'; old='#ad4184'
fig,axs=plt.subplots(2,3,figsize=(12,7.4),gridspec_kw={'width_ratios':[1.55,.9,.95]},layout='constrained')
for i,data in enumerate(json.loads((P/'results.json').read_text())):
    h=data['heading'];sgn=-1 if h==30 else 1;rows=np.array(data['rows']);cvb=photo(h)
    im=cv.cvtColor(cvb,cv.COLOR_BGR2RGB);cam=camera('cameras');rt=solve(cam,rim[h]);color='#ff941f'
    ax,zoom,res=axs[i];ax.imshow(im);ax.axis('off')
    for j,rad in enumerate([13,9.25]):
        ang=np.linspace(0,2*np.pi,200)
        p=np.c_[np.cos(ang)*rad-j*48, np.full(len(ang),27.84919*rt[2]),rad*np.sin(ang)+14.5-j*3.75]
        uv=proj(p,rt,cam); ax.plot(*uv.T,color='#e9bc51',lw=1)
        ax.scatter(*np.array(rim[h][j]).T,c='#f5f0cf',s=3)
    base=np.array(data['base']);new=np.array(data['revised']);opt=np.asarray(data['photo'])
    lo=int(min(base.min(),new.min(),opt.min())-8);hi=int(max(base.max(),new.max(),opt.max())+8)
    y0=rows[0]-3;y1=rows[-1]+3
    ax.add_patch(Rectangle((lo,y0),hi-lo,y1-y0,fill=False,edgecolor='#fff',lw=.9))
    for draw in (ax,zoom):
        if draw==zoom:draw.imshow(im,interpolation='nearest')
        draw.plot(base,rows,color='#e76fbc',lw=1.2)
        draw.plot(new,rows,color='#43e2e1',lw=1.2)
    zoom.plot(opt[:,1],rows,color='#ffffa0',lw=1.0,ls=':');zoom.set_xlim(lo,hi);zoom.set_ylim(y1,y0);zoom.set_aspect('equal')
    zoom.set_xlabel('Rectified x (px)');zoom.set_ylabel('Image row')
    ax.set(xlim=(442,794) if h==30 else (422,770),ylim=(650,265))
    ax.set_title(f'See3CAM · orbit {h}° · '+('right flank' if rt[2]<0 else 'left flank'))
    zoom.set_title('Rear shoulder')
    res.fill_betweenx(rows,np.array(data['pose_min'])-base,np.array(data['pose_max'])-base,color='#d9dfe4',label='Pose sensitivity')
    res.fill_betweenx(rows,np.minimum(opt[:,0],opt[:,2])-base,np.maximum(opt[:,0],opt[:,2])-base,color='#efbc70',alpha=.6,label='25–75% edge')
    res.plot(opt[:,1]-base,rows,color=amber,lw=1.2,label='Photographic edge')
    res.axvline(0,color=old,lw=1.5,label='Original mesh')
    res.plot(new-base,rows,color=sky,lw=1.5,label='Revised mesh')
    res.set_ylim(y1,y0);res.set_xlim(-10,10);res.set_xlabel('x − original contour (px)');res.grid(axis='x',alpha=.16)
    res.set_title('Displacement at fixed row')
    if i==1:res.legend(loc='lower center',bbox_to_anchor=(.4,-.46),ncol=2,fontsize=8)
fig.savefig(P/'see3cam-shoulders.jpg',dpi=190,bbox_inches='tight')
