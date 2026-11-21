"""Check edge ownership in full-rig renders at the rows scored by the camera analyses."""
import json
from pathlib import Path
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.cm import ScalarMappable

def examine(prefix, side, rows):
    crop=json.loads(Path(prefix+'-crop.json').read_text())
    source=crop['scale']
    masks=[cv.imread(prefix+'-'+v+'rig.png') for v in ('old','new')]
    samples=[]
    for row in rows:
        y=round((row-crop['y']+.5)*source-.5)
        readings=[]
        for im in masks:
            covered=np.flatnonzero(im[y,:,2])
            x=covered[0] if side=='left' else covered[-1]
            blue,green,red=im[y,x]
            readings.append((int(x),bool(red==255 and blue==0),float(green)/255))
        samples.append(dict(row=float(row),shell_visible_in_both=all(r[1] for r in readings),
                            exterior_shift_x_px=(readings[1][0]-readings[0][0])/source,
                            vertex_displacement_on_visible_edge_mm=readings[1][2]))
    return crop,masks[-1],samples

if __name__=='__main__':
    cases=json.loads(Path('rows.json').read_text())
    fig,axs=plt.subplots(3,2,figsize=(9,10),constrained_layout=True)
    cmap=plt.get_cmap('magma')
    result={}
    for ax,(prefix,data) in zip(axs.flat,cases.items()):
        crop,im,samples=examine(prefix,data['side'],data['rows'])
        k=crop['scale']
        # Colour only the visible rear cover. Grey shows occluding rig parts.
        shell=(im[:,:,2]==255)&(im[:,:,0]==0); occupied=im[:,:,2]>0
        preview=np.ones((*shell.shape,3),np.uint8)*244
        preview[occupied]=[143,147,150]
        preview[shell]=np.uint8(255*cmap(np.clip(im[:,:,1][shell]/(255*.62404),0,1))[:,:3])
        small=cv.resize(preview,(crop['w'],crop['h']),interpolation=cv.INTER_AREA)
        ax.imshow(small)
        ix=[]
        for s in samples:
            y=round((s['row']-crop['y']+.5)*k-.5); cols=np.flatnonzero(occupied[y])
            ix.append((cols[0] if data['side']=='left' else cols[-1])/k)
        ax.plot(ix,np.array(data['rows'])-crop['y'],color='#27bdad',lw=2)
        ax.set_title(data['title']);ax.axis('off')
        result[prefix]=dict(samples=samples,scale=k,visible=sum(i['shell_visible_in_both'] for i in samples))
    fig.colorbar(ScalarMappable(norm=Normalize(0,.62404),cmap=cmap),ax=axs.ravel().tolist(),shrink=.75,label='Mesh vertex displacement (mm)')
    fig.suptitle('Rear cover displacement in the assembled robot\nTeal marks the tested exterior edge')
    fig.savefig('visibility.jpg',dpi=145)
    Path('visibility-results.json').write_text(json.dumps(result,indent=2)+'\n')
    print([(k,v['visible'],len(v['samples'])) for k,v in result.items()])
