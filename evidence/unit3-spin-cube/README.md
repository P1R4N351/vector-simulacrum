# What the cube felt during three wheelstands with a commanded spin

The physical cube recorded 18,702 three-axis samples in 6,234 Bluetooth packets across W33–W35. All three runs requested a 90° spin on the stand. To compare runs, subtract each run's median pre-mount acceleration vector (−7 to −5 seconds from stand detection) and take the magnitude of the remainder. Results are in g, using 8,192 counts/g. A change in this quantity can come from tilt, translation, or vibration.

![Cube acceleration and robot pitch by trial](cube-spin.png)

| Run | Baseline RMS | Mount peak | Full spin peak | Interior spin RMS | SDK pitch at end-of-wait marker |
|---|---:|---:|---:|---:|---:|
| W33 | 0.0058 g | 3.04 g | 0.223 g | 0.035 g | 70.9° |
| W34 | 0.0043 g | 1.05 g | 1.106 g | 0.041 g | 71.3° |
| W35 | 0.0044 g | 1.98 g | 0.085 g | 0.023 g | 71.6° |

The mount window starts 0.8 s before stand detection and ends at spin start. W35 waited about 1.49 s after stand detection before spinning. The other two began within 0.15 s. The largest packet assigned to each spin arrived within 62 ms of its end. W34's 1.106 g packet arrived just 19 ms before the spin-end marker. Trimming 100 ms from both spin boundaries reduces the largest W34 interior sample to 0.143 g. Keep that boundary impact separate when scoring sustained rotation. Robot pitch remained above 70° at all three log entries named “landed,” and the picked-up flag was set. These labels identify when the runner stopped waiting; they do not establish a physical landing.

The canonical cube sensor at rest produced 0.00394–0.00433 g RMS over 16 seeds, using the same median-centering convention with fixed vertical gravity and 2 s sampling windows. Physical interior-spin RMS was 5–10 times the simulated rest level. The stationary sensor comparison checks the noise floor only. The measured dynamic packet streams are targets for wheelstand contact replay.

Robot and cube times are host receipt/event times relative to stand detection. Three ordered acceleration slots share each packet's timestamp. The plot marks spin command intervals in orange and recorded dismount-step completions with dotted vertical lines. For W34 and W35 the later pulse attempts also produced measurable transients, with late-window peaks of 0.604 and 0.138 g. These data cannot measure fork force or uniquely identify the impact surface.

[Physical packets](cube.csv.gz) · [Robot pitch and gyro](robot.csv.gz) · [Events](events.csv) · [Phase measurements](phase-scores.csv) · [Spin interior](spin-core.csv) · [Simulated rest](sim-rest.csv). Reproduce with python analyze.py and node rest.mjs from this directory with the web dependencies installed.

## Do the recorded cube packets reproduce the reported taps?

The canonical web listener consumed all 6,234 notifications in capture order with a fresh listener for each trial. It emitted one, three and two tap candidates on W33, W34 and W35, respectively. The physical SDK reported the same six events in order. Each matches within 29 ms after applying the earlier **145.906 ms** packet-receipt to callback-receipt estimate. There were no unmatched callbacks or listener candidates within the recorded packet windows. Pairing used order within trial and a ±100 ms gate around that frozen estimate.

![Tap-event timing](tap-timing.png)

On the receiving host, the six SDK callbacks arrived 127–175 ms after the corresponding cube packet. Adding the frozen delay gave 16.1 ms mean absolute timing error. On the robot clock, SDK event stamps were −19 to +36 ms from the corresponding notification's logged robot uptime. Uptime was recorded to 10 ms precision. These clock comparisons describe different stages of event delivery. Applying a 146 ms offset to the robot event stamp would introduce timing error. No delay was fitted on W33–W35.

The replay takes both the measured acceleration slots and the physical cube firmware's tap counter as input. It checks the listener and delivery timing conditional on those inputs. It does not establish that simulated fork contact would generate the same counter transitions. W35's second tap is about 29 seconds after stand detection, outside the commanded spin.

[SDK taps](sdk-taps.csv) · [Radio packet clock pairs](counter-uptime.csv) · [Listener output](replayed-taps.csv) · [Earlier delay evaluation](../unit3-wheelstand/R8/README.md). Run `node listener_replay.mjs > replayed-taps.csv` and `python tap_timing.py` in this directory to reproduce the comparison.
