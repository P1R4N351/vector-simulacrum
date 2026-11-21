"""Sensitivity to the observed 2–3% command-to-rolling-speed difference; no refit."""
import argparse
from dataclasses import replace
from pathlib import Path
from fit_p2 import read_inputs, simulated, dump

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('camera_data', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    runs, _, scores = read_inputs(args.camera_data)
    out = []
    for (seg,start,kt,values), obs in zip(runs,scores):
        adjusted = replace(seg, left=obs['wl'] * (1 if seg.left>0 else -1), right=obs['wr'] * (1 if seg.right>0 else -1))
        for inset in [-1.,0.,3.,3.75]:
            tr,met=simulated(inset,adjusted,start,kt,values)
            out.append(dict(met,target_l=adjusted.left,target_r=adjusted.right,record_yaw=obs['yaw'],record_center_mm=obs['center_mm']))
    dump(args.output/'wheel_speed_sensitivity.csv',out)

if __name__=='__main__': main()
