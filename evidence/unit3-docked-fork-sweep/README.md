# Physical fork sweep on unit 3, November 22

Two back-to-back docked sweeps of the installed printed fork, each stepping through 0, 25, 50, 75 and 100 percent and back down. The middle zero serves as the end of sweep one and the start of sweep two. All 17 lift commands returned success. The robot stayed on the charger in all 555 SDK states, and both wheels reported zero speed throughout. The cube stood clear. These are unloaded motion measurements. After the experiment the robot reported full, on the charger, stationary, with quiet mode active.

[Two-sided video with encoder readout](physical-sweeps.mp4) · [Encoder trace](lift-trace.png) · [First sweep](sweep-1.jpg) · [Repeated sweep](sweep-2.jpg) · [Hold measurements](holds.csv) · [Time series](state.csv.gz)

The 75% holds settled higher on the way up by 1.82 mm and 1.22 mm in sweeps one and two. At 50% the corresponding differences were −0.34 and +2.34 mm. These values are SDK lift heights. Two cycles give a repeatability check and cannot uniquely attribute the differences to gear play, control tolerance, or physical deflection.

All four cameras delivered decoded frames during preflight. The two opposing See3CAMs recorded the motion at 1280 × 720 with V4L2 packet timestamps. The clip uses the closest camera frame at each 100 ms playback sample, with the shelf undistortion, orientation, white balance, denoise and sharpening applied. The encoder readout is interpolated in recorder time. Holds use a median 120–600 ms after action success and the nearest frame to 350 ms after success. Both cameras had a sample within 23 ms of each target on their mapped clocks. That bound covers frame selection only. Cross-host offset and SDK receipt latency are additional uncertainties.

The local archive contains the original A.nut, B.nut, camera clock pairs, and SDK JSONL. Run summarize.py and then make_video.py with the archive directory to regenerate this comparison.

### Frozen Vector 2.0 replay

[Command-aligned overlay](physical-vs-replay.png) · [Per-move errors](replay-errors.csv) · [Replay script](replay.py)

Both sweeps use the measured command schedule, the SDK's linear 32–92 mm target mapping and the requested 0.7 s duration. Replay starts from the recorded head and lift pose and runs continuously across both sweeps. No parameters were fitted. On the second sweep the per-move RMS difference spans 1.01–1.79 mm, including the hold. The largest settled difference repeats at full raise (+2.51 mm on the first sweep, +2.25 mm on the second, simulation minus encoder). At the 75% hold the sign reverses with approach direction on both sweeps. The comparison measures SDK-reported height in recorder time. It does not independently measure fork-tip position, command receipt at the firmware, or loaded grip.

### Does the top-height shortfall transfer?

[Physical endpoint comparison](full-raise-comparison.png) · [Values](full-raise-comparison.csv) · [Reproduce](check_full_raise.py)

Eight earlier unloaded docked full strokes on the same robot settled at 90.81–92.28 mm. Both 75-to-100% raises here ended lower (89.29 and 89.54 mm). A fixed 89.5 mm ceiling would contradict every earlier full stroke. The recordings vary in approach distance, command profile, session and hold timing, so this comparison cannot isolate the cause. The earlier strokes use the median in the last half-second before the following command.

Two overhead-confirmed carries started their outward turn at 91.30 mm (improper engagement) and 90.31 mm (seating unknown). A visually false carry also started at 91.30 mm. These are pre-turn readings after native pickup, with different command histories from the docked holds. Lift height alone did not distinguish actual support from a false carry in these trials.
