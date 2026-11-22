# Twelve docked lift strokes with an independently instrumented cube

A physical Vector 2.0 stayed on the charging platform. Both corrected shelf cameras show the cube separated from the fork. The robot connected the cube under continuous behavior override, set the lift low, and executed six full raise/lower pairs with a one-second pause on either side of each action. The sequence of requested stroke durations was 1.4, 0.6, 0.6, 1.4, 0.6, 1.4 seconds. The SDK encoder and robot state stream and the cube's BLE accelerometer were logged on the pod. Sample times are host receipt times. Three slots share each received cube packet timestamp.

There are 998 robot states and 3,567 cube slots. The encoder measured 59–61 mm of travel. Through the central 80% of each stroke, the fast raises reached 116–129 mm/s with 7.13–7.71 mg cube acceleration magnitude SD, and the slow raises reached 49–50 mm/s with 6.69–6.87 mg. Fast lowers produced 3.04–8.10 mg, showing substantial trial variability. The cube tap counter was constant and the wheels reported zero speed in every scored stroke. These trials establish that the cube sensor can register lift motion across the visible gap on this shared shelf. Shelf and dock transmission cannot be separated in this recording.

The robot was verified charging at 4.07 V with quiet mode enabled, active and muted after disconnect. The corrected dual-camera video is a 12 fps preview; absolute accelerometer phase and encoder-to-camera latency have not been calibrated for this recording.

[Physical traces](physical-profiles.png) · [Both camera views](paired.mp4) · [Per-stroke measurements](moves.csv) · [Cube accelerometer](cube.csv.gz) · [Robot state](state.csv.gz) · [Command log](actions.csv) · [Cube sightings](sightings.csv) · [Analysis](analyze.py)

[Full SDK state](sdk-state.jsonl.gz) also retains the robot's onboard accelerometer, gyro, proximity readings, pose, touch values and last image timestamp alongside the measured lift, head and wheel reports.
