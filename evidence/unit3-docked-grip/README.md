# Docked cube contact with a missing lift tooth

Unit 3 remained on the charger while the experimenter placed a marked face of the cube against its installed printed fork. The experimenter reports that the fork is missing a tooth. Which tooth, the fracture shape, and the date it broke are unverified. The intact STL in earlier contact replays is a comparison geometry. Historical footage establishing a physical carry does not establish full seating.

[Two-sided clip](docked-contact.mp4) · [Contact sequence](contact.jpg) · [Encoder and accelerometer](sensors.png) · [Cube samples](cube.csv) · [Encoder samples](encoder.csv)

The first 25% raise requested a 47 mm target over 1.0 s. The encoder started at 33.02 mm, peaked at 45.42 mm and settled back at 31.32 mm. At 1.518 s the SDK returned MOTOR_STOPPED_MAKING_PROGRESS. The routine stopped; the subsequent 50% and lower commands were never issued. Wheel speed readings stayed at zero. Both cameras show transient cube movement and the cube back down beside the fork. The contact point is occluded.

There were 95 native cube accelerometer notifications (285 samples). Relative to the median pre-command three-axis vector, the largest vector change was 0.010 g before the command and 2.421 g during the action. The latter includes gravity, rotation and transient motion; it is not a grip-force measurement. The three samples in each notification share a packet receipt timestamp. The pre-command packet baseline is short (about 0.20 s). The cameras use V4L2 timestamps with a host clock mapping. Interhost and SDK receipt latency limit event alignment. This single failed hold cannot identify the cause of the stall or quantify a stock fork's capacity.

The archived run.jsonl, packets.tsv, A.nut, B.nut and clock.csv regenerate these exports with analyze.py and make_clip.py. The cube was disconnected and quiet mode restored. Postflight reported full battery, dock contact and stationary robot.
