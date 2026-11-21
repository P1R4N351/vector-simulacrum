from inputs import *
from scipy.ndimage import map_coordinates, gaussian_filter1d
from scipy.optimize import least_squares

# Approximate visible RGB contour in traversal order, clockwise from left side.
outlines={
    'left':[(29,204),(34,185),(46,172),(59,147),(71,135),(85,131),(90,116),(100,102),(115,96),(137,94),(157,95),(183,99),(218,104),(239,110),(253,124),(262,147),(268,173),(269,184)],
    'right':[(64,203),(62,184),(65,157),(70,138),(82,123),(98,113),(124,104),(145,99),(175,94),(195,94),(211,99),(227,110),(237,128),(253,141),(272,160),(281,176)],
    'near':[(325,234),(324,211),(326,181),(334,152),(348,128),(369,109),(401,98),(433,94),(475,91),(493,89),(528,88),(553,84),(578,85),(599,91),(617,104),(632,127)]
}
def register(name,im,z):
    gray=cv.cvtColor(im,cv.COLOR_BGR2GRAY).astype(float)
    p=np.array(outlines[name],float); delta=np.arange(-24,24.25,.25)
    normals=[]; rgb=[]; depths=[]
    for a,b in zip(p[:-1],p[1:]):
        tangent=(b-a)/np.linalg.norm(b-a); normal=np.array([-tangent[1],tangent[0]])
        for fraction in (.25,.5,.75):
            xy=a+(b-a)*fraction
            coords=(xy+delta[:,None]*normal).T[::-1]
            I=map_coordinates(gray,coords,order=1,mode='nearest')
            D=map_coordinates(z,coords,order=1,mode='nearest')
            slope=np.abs(np.gradient(gaussian_filter1d(I,4)))
            choose=np.abs(delta)<=4
            ix=np.flatnonzero(choose)[np.argmax(slope[choose])]
            if slope[ix]<.28:continue
            outside=np.nanmedian(D[:21]);inside=np.nanmedian(D[-21:])
            if not np.isfinite(inside+outside) or outside-inside<45:continue
            threshold=.5*(outside+inside)
            dd=gaussian_filter1d(D,4)
            crosses=np.flatnonzero((dd[:-1]>=threshold)&(dd[1:]<threshold))
            if len(crosses)!=1:continue
            k=crosses[0]
            off=delta[k]+.25*(dd[k]-threshold)/(dd[k]-dd[k+1])
            normals.append(normal);rgb.append(xy+normal*delta[ix]);depths.append(xy+normal*off)
    N=np.array(normals);rgb=np.array(rgb);depths=np.array(depths);diff=np.sum(N*(depths-rgb),axis=1)
    fit=least_squares(lambda shift:N@shift-diff,[0,0],loss='soft_l1',f_scale=2)
    return fit.x,rgb,depths,np.median(np.abs(fit.fun))
