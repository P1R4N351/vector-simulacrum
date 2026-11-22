# Docked cube contact with a missing lift tooth

P2Y8 remained on the charger while the operator placed a marked face of the cube against its installed printed fork. The operator reports that the fork is missing a tooth. Which tooth, the fracture shape, and the date it broke are unverified. The intact STL in earlier contact replays is a comparison geometry. Historical footage establishing a physical carry does not establish full seating.

[Two-sided clip](docked-contact.mp4) · [Contact sequence](contact.jpg) · [Encoder and accelerometer](sensors.png) · [Cube samples](cube.csv) · [Encoder samples](encoder.csv)

The first 25% raise requested a 47 mm target over 1.0 s. The encoder started at 33.02 mm, peaked at 45.42 mm and settled back at 31.32 mm. At 1.518 s the SDK returned MOTOR_STOPPED_MAKING_PROGRESS. The routine stopped; the subsequent 50% and lower commands were never issued. Wheel speed readings stayed at zero. Both cameras show transient cube movement and the cube back down beside the fork. The contact point is occluded.

There were 95 native cube accelerometer notifications (285 samples). Relative to the median pre-command three-axis vector, the largest vector change was 0.010 g before the command and 2.421 g during the action. The latter includes gravity, rotation and transient motion; it is not a grip-force measurement. The three samples in each notification share a packet receipt timestamp. The pre-command packet baseline is short (about 0.20 s). The cameras use V4L2 timestamps with a host clock mapping. Interhost and SDK receipt latency limit event alignment. This single failed hold cannot identify the cause of the stall or quantify a stock fork's capacity.

The archived run.jsonl, packets.tsv, A.nut, B.nut and clock.csv regenerate these exports with analyze.py and make_clip.py. The cube was disconnected and quiet mode restored. Postflight reported full battery, dock contact and stationary robot.

### Optical motion and the free-cube replay

[Glyph, encoder and acceleration overlay](motion-overlay.png) · [Frame measurements](glyph-track.csv) · [Simulation replay](face-replay.png)

Each opposing view tracks a face glyph against the near wheel hub in the published 20 fps clip. Both glyphs reached their highest image position at +1.15 s (16.29 px in A and 12.92 px in B), the same playback sample as the first encoder maximum. At +1.35 s both remained near that peak. At +1.45 s both were within 1.1 px of baseline. The strongest cube acceleration packet arrived at +1.429 s; the action returned its stall result at +1.518 s. These are recorder and playback times, with camera sampling, interhost clock and Bluetooth receipt uncertainties. No force or metric lift distance is inferred from the glyph positions.

The current Vector 2.0 model received the same 47 mm, one-second command. The cube was free and face-square on level ground. Its bounding face began 0, 1 or 2 mm ahead of the projected front of the simulator's three-box fork before settling. The shelf's dock geometry, broken tooth and actual seating depth are unmeasured in this setup. The predicted lift settled at 46.46 mm in every case. The physical lift fell back to 31.32 mm. The zero-gap replay produced a 1.19 mm transient cube-center rise; the other gaps produced at most 0.012 mm. This flags a missing load/release response under these placements. It does not isolate the tooth geometry, compliance or stall controller as the cause. Run track.py, then face_contact.py with the repository on PYTHONPATH.
