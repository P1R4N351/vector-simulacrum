# Opposing shelf camera timing

Two independently started bursts from the opposed See3CAM_24CUG cameras on one host, with the robot stationary. Each process requested 1280×720 MJPEG at 30 fps, copied the V4L2 packet timestamps and recorded 100 packet checksums. A is the native-upright camera and B is the camera that requires a 180° display rotation. The framecrc time base is one microsecond. Sensor exposure midpoint and inter-camera exposure delay have not been measured.

| Burst | Actual average A / B (fps) | Mutual nearest pairs | Median absolute skew | 95th percentile | Maximum |
|---|---:|---:|---:|---:|---:|
| First | 15.28 / 15.09 | 99 | 20.70 ms | 29.51 ms | 32.12 ms |
| Repeat | 15.03 / 15.09 | 99 | 19.29 ms | 31.40 ms | 32.74 ms |

Use individual recorded timestamps when comparing moving landmarks. At 90°/s a 32 ms offset alone corresponds to nearly 3° of rotation. This is a timing example, not a measured cube speed. A common visible light transition would independently measure agreement in image content. These static bursts do not establish stereo geometry or hardware synchronization.

![Mutual nearest timestamp differences](timing.png)

Capture both streams concurrently using the persistent device path for each camera, with `ffmpeg -copyts -f v4l2 -input_format mjpeg -video_size 1280x720 -framerate 30 -i DEVICE -frames:v 100 -c:v copy -f framecrc OUTPUT`. Run `python compare.py` to reproduce the plot from the four packet logs.
