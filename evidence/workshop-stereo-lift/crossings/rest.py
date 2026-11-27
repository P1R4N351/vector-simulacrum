"""Score vertical displacement of the two tread controls in the published docked clip."""
import csv
import cv2
import numpy as np

video = cv2.VideoCapture('rest.mp4')
with open('rest-camera.csv') as src, open('rest-control.csv','w') as dst:
    writer = csv.writer(dst,lineterminator='\n')
    writer.writerow(('relative_s','left_px','right_px','left_corr','right_corr'))
    for i, row in enumerate(csv.DictReader(src)):
        ok, frame = video.read()
        assert ok
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        if i == 0:
            ref = gray
        shifts, scores = [], []
        for x in (60,452):
            match = cv2.matchTemplate(gray[129:174,x-24:x+25],ref[141:162,x-12:x+13],cv2.TM_CCOEFF_NORMED)
            _, score, _, (u,v) = cv2.minMaxLoc(match)
            assert 0 < v < 24 and 0 < u < 24
            a,b,c = match[v-1:v+2,u]
            shifts.append(v-12+.5*(a-c)/(a-2*b+c))
            scores.append(score)
        writer.writerow((row['relative_s'],*shifts,*scores))
    assert not video.read()[0]
video.release()
data = np.loadtxt('rest-control.csv',delimiter=',',skiprows=1)
print(len(data),data[-1,0],np.max(abs(data[:,1:3]),axis=0),data[:,3:].min())
