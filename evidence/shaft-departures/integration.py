"""Determine how much of the camera prediction requires physics beyond shaft integration."""
import csv
import json
import sys
import numpy as np

from score import optical_data
optics,frames=optical_data()
trials=json.load(sys.stdin)
with open("integration.csv","w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["trial","mode","mid_error_mm","camera_frame_rmse_mm"])
    for trial in trials:
        name=trial["name"]
        t,l,r=np.asarray(trial["wheels"]).T
        vel=(l+r)/2
        ref,on,physical=optics[name]
        frames_t,positions=np.asarray(frames[name]).T
        keep=(frames_t>=ref)&(frames_t<=.4)
        for linear in [False,True]:
            area=vel[:-1]*np.diff(t)
            if linear:area+=.5*np.diff(vel)*np.diff(t)
            accum=np.r_[0,np.cumsum(area)]
            def position(at):
                j=np.searchsorted(t,at,side="right")-1
                dt=at-t[j]
                x=accum[j]+vel[j]*dt
                if linear:x+=.5*dt**2*(vel[j+1]-vel[j])/(t[j+1]-t[j])
                return x
            mid=position(on/2)-position(ref)
            rmse=np.mean((position(frames_t[keep])-position(ref)-positions[keep])**2)**.5
            w.writerow([name,"linear" if linear else "shaft",mid-physical,rmse])
