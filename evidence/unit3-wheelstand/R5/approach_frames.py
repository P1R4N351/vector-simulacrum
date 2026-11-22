"""Read clock-registered, rectified R5 shelf frames through frame 85."""
import json
import subprocess
import cv2
import numpy as np
from pickup_onset import action_time, read


def frames(here):
    start = action_time(here/"actions.csv","pickup for inward transport","sec")
    video = here/"carry.mp4"
    probe = subprocess.check_output(["ffprobe","-v","error","-select_streams","v:0","-show_entries",
                                     "frame=best_effort_timestamp_time","-of","json",str(video)])
    t = np.array([float(row["best_effort_timestamp_time"]) for row in json.loads(probe)["frames"]])
    t += read(here/"video-registration.csv")["offset_s"][0] - start
    K = np.array([[340.,0,209.5],[0,340.,119.5],[0,0,1]])
    raw_k = np.array([[249.2,0,480],[0,249.2,270],[0,0,1]])
    rot = cv2.Rodrigues(np.radians([24.,0,0]))[0] @ cv2.Rodrigues(np.radians([0,49.,0]))[0]
    maps = cv2.fisheye.initUndistortRectifyMap(raw_k,np.zeros(4),rot,K,(420,240),cv2.CV_32FC1)
    cap = cv2.VideoCapture(str(video))
    imgs = []
    for i in range(86):
        ok, frame = cap.read()
        assert ok
        rect = cv2.bilateralFilter(cv2.remap(frame,*maps,cv2.INTER_LINEAR),5,18,18)
        imgs.append(cv2.addWeighted(rect,1.25,cv2.GaussianBlur(rect,(0,0),1),-.25,0))
    cap.release()
    return t[:86], K, imgs
