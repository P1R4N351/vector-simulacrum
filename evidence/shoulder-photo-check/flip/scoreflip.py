"""Evaluate the two visible rear shell profiles in native Flip pixels."""
from fitflip import *
import os
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import PchipInterpolator
import matplotlib.pyplot as plt

meshdir=Path(os.environ.get('MESH_EXPORTS','.'))
def mesh(file):return np.array(json.loads((meshdir/file).read_text())).reshape(141,85,3)*10-[2.947435,0,0]
old,new=mesh('pre.json'),mesh('post.json')
# Independent visual tracing in native RGB, seeded in a 0.75x crop of each 8160-pixel exposure.
# Clip to clean shoulder against background above wheels and below top fittings.
guide={60:([200,250,300,350,400,450,500,550,590],[121,104,95,89,91,82,84,90,94],293),
       195:([200,250,300,350,400,450,500,550,590],[50,67,81,87,97,96,95,91,85],685),
       105:([200,250,300,350,400,450,500,550,590],[156,142,129,116,106,100,96,95,95],203),
       180:([200,250,300,350,400,450,500,550,590],[42,56,68,78,86,92,95,96,95],581)}
def traced(im,n):
    y,x,c=guide[n]; ys=np.arange(3000+max(y[0],240)/.75,3000+min(y[-1],560)/.75,8)
    seed=PchipInterpolator(np.array(y)/.75+3000,np.array(x)/.75+8*c-120)(ys)
    xspan=np.arange(-30,31)
    data=[]
    for ch in [cv.cvtColor(im,cv.COLOR_BGR2GRAY),*cv.split(im)]:
        # Vertical median removes isolated hairs and compression ringing.
        profiles=np.array([np.median(ch[round(y)-3:round(y)+4,round(x)-30:round(x)+31],axis=0) for x,y in zip(seed,ys)])
        for sigma in [2,4,7]:
            grad=abs(gaussian_filter1d(profiles.astype(float),sigma,axis=1,order=1))
            grad[:,:9]=0;grad[:,-9:]=0
            data.append(seed.round()+grad.argmax(1)-30)
    arr=np.array(data);return ys,np.median(arr,axis=0),np.percentile(arr,[10,90],axis=0)
def outline(v,R,t,f,side,ys):
    uv=proj(v,R,t,f)
    # True mesh silhouette. A horizontal plane cuts projected triangle edges;
    # row extrema are the shell's left/right occluding contour.
    aa=np.concatenate((uv[:,:-1].reshape(-1,2),uv[:-1].reshape(-1,2),uv[:-1,1:].reshape(-1,2)))
    bb=np.concatenate((uv[:,1:].reshape(-1,2),uv[1:].reshape(-1,2),uv[1:,:-1].reshape(-1,2)))
    result=[]
    for y in ys:
        sel=(np.minimum(aa[:,1],bb[:,1])<=y)&(np.maximum(aa[:,1],bb[:,1])>y)
        a,b=aa[sel],bb[sel]
        result.append(side*np.max(side*(a[:,0]+(b[:,0]-a[:,0])*(y-a[:,1])/(b[:,1]-a[:,1]))))
    return np.array(result)
def get_pose(p,es,n,f,radii):
    samples=[np.c_[edge,np.ones(len(edge))].T for edge in es]
    opt=least_squares(residual,p,args=(samples,48,f,np.array(radii)),x_scale='jac')
    return world(opt.x,n)[:2]
def family(d,n,ys):
    pts=[np.array(a) for a in d['edges']];_,p=fit(pts,48)
    baseR,baset,_=world(p,n);sign=1 if n in (180,195) else -1
    def curve(m,r=baseR,t=baset,f=FOCAL):return outline(m,r,t,f,sign,ys)
    A=curve(old);B=curve(new);variants=[]
    for sector in range(8):
        # Simultaneously leave out the same octant on the two rim faces.
        partial=[]
        for ed in pts:
            a=ed-ed.mean(0);octants=np.floor((np.arctan2(a[:,1],a[:,0])+np.pi)*8/(2*np.pi)).astype(int)%8
            partial.append(ed[octants!=sector])
        r,t=get_pose(p,partial,n,FOCAL,[13,9.25]);variants.append(curve(old,r,t))
    for focal,radii in [(FOCAL*.95,[13,9.25]),(FOCAL*1.05,[13,9.25]),(FOCAL,[12.75,9])]:
        r,t=get_pose(p,pts,n,focal,radii);variants.append(curve(old,r,t,focal))
    return A,B,np.array(variants)
if __name__=='__main__':
    fig,axes=plt.subplots(2,3,figsize=(13,8),constrained_layout=True)
    table=[]
    broad='--broad' in sys.argv
    tag='-broad' if broad else ''
    for row,n in enumerate([60,195] if broad else [105,180]):
        filename=('S-' if broad else '')+f'ring{n}-main.jpg'
        im=cv.imread(str(raw/filename)); record=json.loads(Path(f'flip-{n}.json').read_text())
        ys,edge,band=traced(im,n);A,B,other=family(record,n,ys)
        # Distances measured along the old projected contour normal.
        norm=1/np.hypot(1,np.gradient(A,ys))
        separation=(B-A)*norm; pose=np.max(abs(other-A),axis=0)*norm
        pick=np.max(abs(band-edge),axis=0)*norm
        audit=Path(os.environ.get('SHOULDER_VISIBILITY',Path(__file__).resolve().parents[1]/'visibility/visibility-results.json'))
        visible={r['row'] for r in json.loads(audit.read_text())[f'flip{n}']['samples'] if r['shell_visible_in_both']}
        keep=np.isin(ys,list(visible)); dropped=ys[~keep].tolist()
        full_y=ys.copy()
        ys,edge,band,A,B,other,norm,separation,pose,pick=(v[...,keep] for v in (ys,edge,band,A,B,other,norm,separation,pose,pick))
        def plotted(v):
            # Break the line at samples owned by other exterior geometry.
            out=np.full(len(full_y),np.nan);out[keep]=v
            return out
        table.append(dict(rows_rejected_occluded=dropped,file=filename,native_size=[8160,6120],rim_rms_px=record['rms'],
            rows_native_px=ys.tolist(),observed_edge_x=edge.tolist(),old_x=A.tolist(),new_x=B.tolist(),
            normal_separation_px=separation.tolist(),pose_sensitivity_px=pose.tolist(),pick_sensitivity_px=pick.tolist(),
            median_absolute_residual_old_new_px=[np.median(abs((A-edge)*norm)),np.median(abs((B-edge)*norm))],
            rows_separation_gt_pose_plus_pick=int(np.sum(abs(separation)>pose+pick))))
        ov,ax,margin=axes[row]
        ox,oy=(1100,2180) if n==105 else (1300,2180)
        xstop=5080 if n==105 else 5100;ystop=5420
        if broad:ox,xstop=(2080,6260) if n==60 else (1590,5800)
        ov.imshow(cv.cvtColor(cv.resize(im[oy:ystop,ox:xstop],(640,520)),cv.COLOR_BGR2RGB),
                  extent=(ox,xstop,ystop,oy))
        for es in record['edges']:ov.scatter(*np.array(es).T,s=2,color='#bd98ff')
        for axs in (ov,ax):
            if axs==ax:
                x0=int(min(edge.min(),A.min(),other.min())-40);x1=int(max(edge.max(),B.max(),other.max())+40)
                y0,y1=round(ys.min()-5),round(ys.max()+5)
                axs.imshow(cv.cvtColor(im[y0:y1,x0:x1],cv.COLOR_BGR2RGB),extent=(x0,x1,y1,y0),aspect='auto')
                axs.fill_betweenx(full_y,plotted(other.min(0)),plotted(other.max(0)),color='#b6b9be',alpha=.28)
            axs.plot(plotted(A),full_y,color='#f6a43d',lw=1.4,label='Original')
            axs.plot(plotted(B),full_y,'--',color='#38deff',lw=1.5,label='Revised')
            axs.plot(plotted(edge),full_y,color='#a1ef76',lw=.9,label='Photo edge')
            axs.axis('off')
        ov.set_title(f'Flip {n}° | native rim samples')
        ax.set_title('Shoulder detail (horizontal enlarged)')
        margin.plot(full_y,plotted(abs(separation)),label='Mesh separation',color='#087b95')
        margin.plot(full_y,plotted(pose),label='Pose sensitivity',color='#7759a5')
        margin.plot(full_y,plotted(pick),label='Edge sensitivity',color='#31864e')
        margin.set(xlabel='Native image row (px)',ylabel='Contour-normal distance (native px)')
        margin.legend(fontsize=8)
        for k,v in table[-1].items():
            if 'sensitivity_px' in k or 'separation_px' in k:print(n,k, np.round(np.percentile(abs(np.array(v)),[5,50,95]),2))
        table[-1]['rim_octant_only_median_sensitivity_px']=float(np.median(np.max(abs(other[:8]-A),axis=0)*norm))
        print(n,table[-1]['median_absolute_residual_old_new_px'],table[-1]['rows_separation_gt_pose_plus_pick'])
    fig.legend(*axes[0,1].get_legend_handles_labels(),loc='outside lower center',ncol=3)
    fig.savefig('flip-shoulders'+tag+'.jpg',dpi=160)
    Path('flip-results'+tag+'.json').write_text(json.dumps(table,indent=2)+'\n')
