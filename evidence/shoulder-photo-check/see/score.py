import json
from scipy.ndimage import gaussian_filter1d,map_coordinates
import numpy as np, cv2 as cv
from analysis import P, M, rim, first_pass, camera, solve, proj, photo as load_photo

def row_edges(mesh,rt,cam,ys,sign):
    q=proj(mesh,rt,cam).reshape(141,85,2)
    # Mesh rows, columns and the original diagonal shared by each cell's triangles.
    segs=[(q[1:],q[:-1]),(q[:,1:],q[:,:-1]),(q[1:,:-1],q[:-1,1:])]
    x0,y0,x1,y1=np.concatenate([np.concatenate([a,b],axis=2).reshape(-1,4) for a,b in segs]).T
    result=[]
    for y in ys:
        ok=((y1-y)*(y0-y)<=0)&(y0!=y1)
        x=x0[ok]+(y-y0[ok])*(x1[ok]-x0[ok])/(y1[ok]-y0[ok])
        result.append(sign*np.max(sign*x))
    return np.array(result)

def photo_edge(im,h,ys):
    # Optical edge from local inside/outside colors, anchored independently in native image.
    seed=np.interp(ys,[398,425] if h==30 else [426,454],[489,486] if h==30 else [738,745])
    dist=np.arange(-13.,13.01,.1)
    sign=-1 if h==30 else 1
    x=seed[:,None]+sign*dist
    ys_grid=np.broadcast_to(ys[:,None],x.shape)
    colors=np.stack([map_coordinates(im[:,:,channel].astype(float), [ys_grid,x],order=1) for channel in range(3)],axis=-1)
    inside=colors[:,:30].mean(1);outside=colors[:,-30:].mean(1);d=outside-inside
    score=np.einsum('ijk,ik->ij',colors-inside[:,None],d)/(np.sum(d*d,axis=1)[:,None])
    result=[]
    for row in score:
        xs=[]
        for t in [.25,.50,.75]:
            crossings=np.where((row[:-1]<=t)&(row[1:]>t))[0]
            if not len(crossings):raise RuntimeError('no image edge')
            idx=crossings[np.argmin(abs(dist[crossings]))];xs.append(dist[idx]+.1*(t-row[idx])/(row[idx+1]-row[idx]))
        result.append(xs)
    return seed[:,None]+sign*np.array(result)

def score(h):
    im=load_photo(h)
    ys=np.arange(398,426) if h==30 else np.arange(426,455)
    sign=-1 if h==30 else 1
    cam=camera('cameras');rt=solve(cam,rim[h])
    original,updated=[row_edges(v,rt,cam,ys,sign) for v in M]
    photo=photo_edge(im,h,ys)
    poses={}
    score_changes={}
    def add(name,cam,picks,**kwargs):
        rt=solve(cam,picks,**kwargs)
        poses[name]=row_edges(M[0],rt,cam,ys,sign)
        second=row_edges(M[1],rt,cam,ys,sign)
        score_changes[name]=float(np.mean(abs(second-photo[:,1]))-np.mean(abs(poses[name]-photo[:,1])))
    add('first rim reading',cam,first_pass[h])
    add('narrow face diameters',cam,rim[h],diam=(25.5,18))
    for tag in ('omit-ring180','face-land'):add(tag,camera(tag),rim[h])
    for j in [0,1]:
        n=len(rim[h][j])
        for block,indices in enumerate(np.array_split(np.arange(n),4)):
            arcs=[np.arange(len(r)) for r in rim[h]]
            arcs[j]=np.setdiff1d(arcs[j],indices)
            add(f'wheel {j} drop arc {block}',cam,rim[h],arcs=arcs)
    for axis in (0,1):
        for shift in (-2,2):
            dp=np.zeros(2);dp[axis]=shift
            add(f'pixel axis {axis} shift {shift}',cam,[np.asarray(p)+dp for p in rim[h]])
    alternatives=np.stack(list(poses.values())+[original])
    # Compare horizontal positions at the same observed scanline, in rectified px.
    spread=np.max(abs(alternatives-original),axis=0)
    edgehalf=abs(photo[:,2]-photo[:,0])/2
    return dict(heading=h,rim_rms_px=rt[-1],rows=ys.tolist(),base=original.tolist(),revised=updated.tolist(),photo=photo.tolist(),
                pose_min=alternatives.min(0).tolist(),pose_max=alternatives.max(0).tolist(),pose_sensitivity_by_variant={k:float(np.max(abs(v-original))) for k,v in poses.items()},
                mae_change_under_pose_variants_px=score_changes,
                median_shift=float(np.median(abs(updated-original))),max_shift=float(np.max(abs(updated-original))),
                median_edge_halfwidth=float(np.median(edgehalf)),median_pose_perturbation=float(np.median(spread)),
                rows_clearing_pose_and_edge=int(sum(abs(updated-original)>spread+edgehalf)),
                baseline_mae=float(np.mean(abs(photo[:,1]-original))),revised_mae=float(np.mean(abs(photo[:,1]-updated))))
if __name__=='__main__':
    out=[score(h) for h in (30,150)]
    (P/'results.json').write_text(json.dumps(out,indent=2))
    for row in out:print({k:v for k,v in row.items() if not isinstance(v,(dict,list))})
