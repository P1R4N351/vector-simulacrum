"""Use settled glyph height and vertical edges to check camera extrinsics."""
import csv
import json
from pathlib import Path
import cv2
import numpy as np
from geometry import camera, HERE

picks = json.loads((HERE.parent / 'p2y8-paired-cube-push/endpoint-picks.json').read_text())
square = 11.5*np.array([[-1.,-1,0],[1,-1,0],[1,1,0],[-1,1,0]])
K = np.array([[camera['fx_px'],0,camera['cx_px']],[0,camera['fy_px'],camera['cy_px']],[0,0,1.]])
with (HERE/'shelf.csv').open('w') as out:
    writer = csv.writer(out)
    writer.writerow(['trial','time','glyph_center_z_mm','inferred_camera_pitch_deg','nominal_pitch_deg'])
    for pick in picks:
        if pick['camera'] != 'on':
            continue
        ok, rv, tv = cv2.solvePnP(square, np.asarray(pick['quad']), K, None)
        assert ok
        head = pick['state']['head']
        pitch = head - np.deg2rad(4)
        lens_z = 49 + 18.84*np.sin(head) - 7.96*np.cos(head)
        glyph_z = lens_z + np.dot([0,-np.cos(pitch),np.sin(pitch)],tv.ravel())
        down = cv2.Rodrigues(rv)[0][:,1]
        inferred_pitch = np.arctan2(-down[2], down[1])
        writer.writerow([pick['trial'],pick['time'],glyph_z,np.rad2deg(inferred_pitch),np.rad2deg(pitch)])

# Freeze mount corrections using the first two trials. Score all remaining picks.
from geometry import silhouette, edges
measurements = np.genfromtxt(HERE/'shelf.csv', delimiter=',',names=True)
train = measurements['trial'] <= 2
correction = np.deg2rad(np.median((measurements['inferred_camera_pitch_deg']-measurements['nominal_pitch_deg'])[train]))
heights = []
for pick in (p for p in picks if p['camera']=='on'):
    _, _, tv = cv2.solvePnP(square, np.asarray(pick['quad']), K, None)
    h = pick['state']['head']
    angle = h - np.deg2rad(4) + correction
    heights.append(49 + 18.84*np.sin(h) - 7.96*np.cos(h) + np.dot([0,-np.cos(angle),np.sin(angle)],tv.ravel()))
dz = 22 - np.median(np.asarray(heights)[train])
print('Pitch (deg), height (mm) adjustments',np.rad2deg(correction), dz)
print('Held-out glyph heights',np.asarray(heights)[~train]+dz)
for trial,l in [(4,67.467987),(5,66.160782)]:
    print(trial, edges(silhouette(l,-0.212401658,correction,dz)))
