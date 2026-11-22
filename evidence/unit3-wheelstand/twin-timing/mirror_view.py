"""Mirror comparison of the fixed twin shelf cameras in corrected 1000x440 coordinates."""
import cv2
import numpy as np

# Equivalent silhouette landmarks in opposite views. These compare the apparent
# shape, including opposite cube faces; they are not stereo point correspondences.
A = np.float32([(521,262),(659,266),(525,386),(650,394),
                (84,186),(168,166),(106,285),(180,253),(371,296),(312,366)])
B = np.float32([(480,177),(368,181),(479,277),(375,281),
                (948,102),(857,94),(919,213),(835,187),(640,239),(697,302)])
# Physical corners of the top glyph border, clockwise as viewed by A.
# In B both horizontal and depth order reverse on this common physical plane.
TOP_A = np.float32([(552,233),(615,234),(621,254),(551,253)])
TOP_B = np.float32([(456,172),(400,173),(406,159),(457,158)])


def comparison(a, b):
    if a.shape != (440,1000,3) or b.shape != a.shape:
        raise ValueError('Mirror registration requires both full corrected shelf frames')
    transform, _ = cv2.findHomography(B, A, method=0)
    aligned = cv2.warpPerspective(b, transform, (1000,440))
    valid = cv2.warpPerspective(np.full(b.shape[:2],255,np.uint8),transform,(1000,440)) == 255
    overlay = cv2.addWeighted(a,.5,aligned,.5,0)
    overlay[~valid] = 0
    square = np.float32([(0,0),(139,0),(139,139),(0,139)])
    tops = [cv2.warpPerspective(img, cv2.getPerspectiveTransform(points,square),(140,140))
            for img,points in [(a,TOP_A),(b,TOP_B)]]
    panels = []
    top_overlay = cv2.addWeighted(tops[0],.5,tops[1],.5,0)
    for frame,top,label in zip((a,aligned,overlay),(*tops,top_overlay),
                               ('A','B mirrored','50/50 overlay')):
        panel = frame[126:420,25:725].copy()
        cv2.rectangle(panel,(0,0),(160,26),(0,0,0),-1)
        cv2.putText(panel,label,(6,19),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
        panel[:100,345:445] = cv2.resize(top,(100,100),interpolation=cv2.INTER_AREA)
        panels.append(panel)
    return np.vstack(panels)
