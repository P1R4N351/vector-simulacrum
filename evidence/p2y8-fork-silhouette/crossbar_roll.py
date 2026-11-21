"""Compare projected crossbar slope with two independent raised-fork frames."""
import csv
import cv2
import numpy as np
from reach_silhouette import HERE, matched

if __name__ == "__main__":
    cols = np.arange(515,626,10)
    with (HERE / "crossbar-roll.csv").open("w") as f:
        writer = csv.writer(f,lineterminator="\n")
        writer.writerow(["trial","threshold","roll_deg","observed_span_px","sim_span_px"])
        for trial in (4,5):
            gray = cv2.imread(str(HERE / f"frame{trial}.png"),0)
            profiles = [np.median(gray[:,x-510-4:x-510+5],axis=1) for x in cols]
            for fraction in (.35,.5,.65):
                points = []
                for profile in profiles:
                    dark, bright = np.median(profile[200:235]),np.median(profile[280:294])
                    points.append(230+np.flatnonzero(profile[230:296] > dark+fraction*(bright-dark))[0])
                observed = np.polyfit(cols,points,1)
                center = np.polyval(observed,570)
                for angle in (-4,-2,-1.5,-1,0,2,4):
                    _, mask = matched(round(center),0,5.16,-.52,roll=angle)
                    pred = [np.flatnonzero(mask[:,x])[-1] for x in cols]
                    writer.writerow([trial,fraction,angle,110*observed[0],110*np.polyfit(cols,pred,1)[0]])
