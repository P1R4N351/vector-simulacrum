# Two-camera timing before a cube trial

Both UVC streams delivered frames during 2.899 seconds of common coverage. The opposite Arducam requested MJPEG 640×480 at 30 Hz and delivered 180 frames averaging 19.96 Hz, with a maximum timestamp gap of 52.03 ms. The See3CAM requested MJPEG 1280×720 at 15 Hz and delivered 45 frames averaging 15.18 Hz, with a maximum gap of 66.01 ms. The first camera opened about one second earlier. Two subsequent attempts with the same format and rate failed at Arducam stream startup. Check for advancing timestamps on both streams before issuing a movement command.

Times come from V4L2 packet timestamps captured on one host using ffmpeg with -copyts and the framecrc muxer. Both time bases were one microsecond. The table subtracts a single shared origin. The short static test measures frame delivery and cannot determine relative exposure delay. Use a common visible motion to check optical alignment during the next trial. These timing measurements do not score the physics model.

[Per-frame intervals](intervals.png) · [Timestamps](frames.csv)
