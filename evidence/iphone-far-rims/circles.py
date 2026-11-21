import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

def circlefit(ellipses, f, pp):
    # Fit two parallel circles without using wheelbase or any depth samples.
    angle=np.linspace(0,2*np.pi,72,endpoint=False)
    unit=np.stack((np.cos(angle),np.sin(angle),np.zeros_like(angle)),axis=1)
    starts=[]; obs=[]
    for (ellipse,_),diameter in zip(ellipses,(18.5,26),strict=True):
        uv,axes,theta=ellipse
        z=np.mean(f)*diameter/max(axes)
        starts.extend(np.r_[(uv-pp)/f,1]*z)
        a=np.deg2rad(theta)
        turn=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
        obs.append((np.array(uv),np.array(axes)/2,turn,diameter))
    def residual(v):
        rotated=Rotation.from_rotvec([*v[6:],0]).apply(unit)
        errors=[]
        for point,(uv,axes,turn,d) in zip(v[:6].reshape(2,3),obs):
            xyz=point+rotated*d/2
            pixels=xyz[:,:2]/xyz[:,2,None]*f+pp
            errors.extend((np.linalg.norm((pixels-uv)@turn/axes,axis=1)-1)*np.mean(axes))
        return errors
    fits=[least_squares(residual,[*starts,a,b]) for a in (-.4,.4) for b in (-.4,.4)]
    best=min(fits,key=lambda fit:fit.cost)
    return best.x[:6].reshape(2,3),np.sqrt(np.mean(best.fun**2))
