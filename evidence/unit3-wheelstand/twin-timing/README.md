# Opposing shelf camera timing

Two independently started bursts from the opposed See3CAM_24CUG cameras on one host, with the robot stationary. Each process requested 1280×720 MJPEG at 30 fps, copied the V4L2 packet timestamps and recorded 100 packet checksums. A is the native-upright camera and B is the camera that requires a 180° display rotation. The framecrc time base is one microsecond. Sensor exposure midpoint and inter-camera exposure delay have not been measured.

| Burst | Actual average A / B (fps) | Mutual nearest pairs | Median absolute skew | 95th percentile | Maximum |
|---|---:|---:|---:|---:|---:|
| First | 15.28 / 15.09 | 99 | 20.70 ms | 29.51 ms | 32.12 ms |
| Repeat | 15.03 / 15.09 | 99 | 19.29 ms | 31.40 ms | 32.74 ms |

Use individual recorded timestamps when comparing moving landmarks. At 90°/s a 32 ms offset alone corresponds to nearly 3° of rotation. This is a timing example, not a measured cube speed. A common visible light transition would independently measure agreement in image content. These static bursts do not establish stereo geometry or hardware synchronization.

![Mutual nearest timestamp differences](timing.png)

Capture both streams concurrently using the persistent device path for each camera, with `ffmpeg -copyts -f v4l2 -input_format mjpeg -video_size 1280x720 -framerate 30 -i DEVICE -frames:v 100 -c:v copy -f framecrc OUTPUT`. Run `python compare.py` to reproduce the plot from the four packet logs.

### Visible timing check with the cube LEDs

With the cube centered, both cameras recorded three green pulses, including all three falling edges. The battery-powered cube was driven briefly from the camera host over its BLE application light characteristic and left dark, then disconnected. The shelf lamp was untouched. We used the first pulse to locate the strongest local increase in green relative to red and blue in each image, then scored all three pulses at those fixed locations. For each transition, the last frame on one side of half brightness and the first frame on the other side bracket the visible change. All six pairs of camera brackets overlapped. B-minus-A midpoint differences ranged from −23.8 to +47.6 ms, so a constant correction inferred from this run is unwarranted. Frames are free-running, and command acknowledgement is only the plotting origin.

[Three pulses on both cameras](cube-led-pulses.png) · [Six edge brackets](cube-led-edges.csv) · [Analysis script](cube_pulses.py)

### Default inspection view for the opposed cameras

Use `python live_view.py HOST preview.jpg` for a fresh upright pair. Routine visual inspection, LED pulse extraction and glyph tracking use the same shelf view. Use the original recordings explicitly for optical calibration or diagnosis. Preserve source timestamps. The 1280×720 camera inputs are mapped to a 1000×440 rectilinear shelf view with a downward virtual pitch and ceiling crop. The second camera is rotated 180°. Color gains come from a bare shelf patch in the reference frame and stay fixed throughout a clip. A fixed midtone lift brings out shaded features. A mild bilateral denoise and a 0.30-strength unsharp mask operate on luminance within each individual frame. The sharpening contribution is capped at four levels on an eight-bit scale. Replaying the centered LED recording gave identical crossing frames for all six transitions before and after adding these filters.

The equidistant focal length transfers the earlier single-camera shelf estimate at the new resolution. It is an approximate viewing correction. Absolute distance and stereo pose need separate calibration of the two mounted lenses. The shelf is a relative neutral reference; these views have no color-chart calibration. Refresh the reference after a lighting or camera change and confirm the patch still shows bare shelf.

![Corrected shelf preview](corrected-preview.jpg)

We reran the three-pulse test through the corrected view. All six edge brackets agree with the earlier scoring. In the four LED-off windows the 95th-percentile apparent motion of stationary glyph corners ranged from 1.01 to 1.71 corrected pixels on A and 0.21 to 0.30 on B. With the LEDs on, A lost all 60 attempted corner matches under a one-pixel round-trip consistency check. B retained all 56. Use lit corners for timing and unlit glyphs for position on A. Neither pixel result measures millimetres or establishes accuracy on moving targets.

[Glyph landmarks and residuals](glyph-tracking.png) · [Per-frame scores](glyph-tracking.csv) · [Analysis](glyph_tracking.py)

### Does the LED marker perturb the cube accelerometer at rest?

Two separate direct BLE sessions each alternated four short green dwells with five dark dwells. For each light write we scored samples received 150–700 ms after acknowledgement. For each green dwell we subtracted the average of its two neighboring dark-window means on each axis. This suppresses slow baseline drift. All four corners used the same light command as the camera timing experiment. These sensor-only repeats did not independently film or time diode emission.

| Session | Three-sample packets | Tap-counter changes | Largest absolute paired axis shift |
|---|---:|---:|---:|
| First | 213 | 0 | 1.20 mg |
| Independent repeat | 224 | 0 | 0.60 mg |

Across both sessions the nine received rail reports all decoded to 1.561 V. Their sparse timing cannot resolve supply transients. The first three green dwells in the first session had positive X and Y differences; this did not repeat consistently. These records support using the marker for stationary synchronization on this cube. They do not establish a bound during switching or mechanical impacts. Both sessions ended dark and disconnected.

[First sensor trace](led-accel.png) · [Repeat](led-accel-repeat.png) · [First packet log](led-accel.csv) · [Repeat log](led-accel-repeat.csv) · [Capture](led_accel_capture.py) · [Analysis](led_accel.py). Run the analysis once without flags and once with `--repeat`.

### Periodic gaps persist when the camera host connects directly to the cube

The two LED recordings and a separate ten-second stationary recording with no light writes used the camera host as the BLE central. All three show intervals over 150 ms between consecutive acceleration packet receipts. The period of 1.22746 s was fitted previously on the robot's rest capture. Here each session's first gap return establishes phase. The periods and widths were never fitted on these three captures.

| Direct session | Acceleration packets | Gaps | Median gap | Max return prediction error |
|---|---:|---:|---:|---:|
| Green pulses | 213 | 6 | 262.1 ms | 43.1 ms |
| Green repeat | 224 | 6 | 262.1 ms | 25.7 ms |
| No light writes | 285 | 7 | 261.9 ms | 43.5 ms |

Of the voltage reports received from the start of the first complete gap through the last acceleration packet, all nine were within 25 ms of a gap return. Ten of the 19 gaps lacked a nearby received voltage report. Before the first complete gap, each capture also had one voltage report. The published firmware calls the battery ADC before the accelerometer in each app tick and measures battery every 100 ticks. The timer schedules the next tick after the current callback returns. Battery sampling or how the resulting notification is delivered is a plausible contributor; receipt timestamps cannot distinguish a pause in sampling from a delay or loss in transmission. The gap recurs even when the robot, SDK and light commands are absent. Direct capture alone will not fill the missing interval in a roll replay.

[Voltage receipts and predicted returns](gap-voltage.png) · [Dark control packets](dark-direct.csv) · [Acquisition](dark_capture.py) · [Analysis](gap_voltage.py) · [Cube ADC firmware](https://github.com/kercre123/victor/blob/snowboy/robot/cube_firmware/app/adc.c)

### Optical probe of the cube's application clock

One BLE command started an autonomous green/dark loop on the stationary shelf cube. The firmware advances the animation every third application tick; each color was set to hold for five animation steps (nominally 150 ms). The host sent no further light writes during the eight-second observation. The cameras used the corrected shelf pipeline. We located the modulated corner near the prior cube position from the first two seconds, then held that patch fixed.

Six acceleration receipt gaps lasted 243–281 ms. Both cameras covered the first four gaps with 4–7 frames per camera per gap. Away from gaps the median interval between half-brightness edge midpoints was 157 ms in A and 158 ms in B. At the midpoints of the four jointly covered gaps, the visible color dwells lasted 310–373 ms in A and 355–434 ms in B. There were zero same-polarity transitions with overlapping exposure brackets wholly inside those four gaps. A alone covered the last two gaps. Camera B reached its 170-frame limit earlier because it delivered frames faster in this lighting.

The visible holds lengthen when accelerometer receipts stop. Delaying only the host callback while the cube continues its normal animation cannot explain that observation. This supports a delay in application progress or LED updating inside the cube. The timing cannot identify the specific blocking firmware operation or show what the accelerometer chip did internally. The cube finished dark and disconnected.

[Independent optical clocks and packet gaps](blink-clock.png) · [Per-gap coverage and dwell estimates](blink-clock-gaps.csv) · [Corrected brightness traces](blink-clock-traces.csv) · [BLE receipts](blink-clock-receipts.csv) · [Capture](blink_clock_capture.py) · [Analysis](blink_clock.py)

### Predicting later blinks from rest accelerometer timing

The independent rest recording supplied the 1.227461 s period and 230 ms excess gap (median long interval minus median ordinary interval). The first returning packet set the pause phase. The midpoint of camera A's first falling edge set the optical phase for both cameras. All models used the programmed 150 ms hold. The uniform slowdown used the same average clock rate as the pause model. Edges were paired by ordinal and polarity, and scoring began after the first gap return. Each camera used its own early brightness threshold on the previously extracted patch.

| Camera | Later edges | Continuous median absolute error | Uniform slowdown | Periodic pause | Pause predictions within frame bracket |
|---|---:|---:|---:|---:|---:|
| A | 36 | 680 ms | 84 ms | 30 ms | 17/36 |
| B | 25 | 459 ms | 73 ms | 28 ms | 9/25 |

Both cameras filmed the same cube run. B independently checks optical timing, while the run tests transfer from the earlier rest recording. The pause model still placed the falling edge around 3.3 s one pause too late (251–254 ms). A saw a similar 236 ms error near 7 s after B's capture ended. This makes a hard pause boundary inferred from BLE receipt time unreliable for predicting individual edge order near that boundary. The score covers the visible transitions and does not establish internal accelerometer sampling during the gaps.

[Edge timing overlay](blink-prediction.png) · [Every observed bracket and prediction](blink-prediction.csv) · [Reproduce the prediction](blink_prediction.py)
