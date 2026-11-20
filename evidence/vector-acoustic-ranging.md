# Vector acoustic ranging feasibility

A stationary, fixed-head, single-wall range cue is plausible **if simultaneous microphone samples survive speaker playback**. Bearing also needs synchronized channels with known positions and channel order. None of the archived motion recordings contains a microphone waveform. The checks below used source inspection and synthetic signals only.

## What Vector and Susurrus expose

- Anki’s protocol specifies a four-microphone direction estimator with 12 sectors, raw microphone sampling at 15,625 Hz and processed audio at 16 kHz. Its external audio feed carries mono PCM and direction metadata. Those fields alone cannot supply four independent arrival times. The Python playback path accepts mono 16-bit PCM, 8–16.025 kHz. I inspected the installed wire-pod SDK on the paired SDK host without connecting to Vector: the robot's audio-feed property stores a boolean and has a TODO for implementing feed enablement. [Anki protocol](https://github.com/anki/vector-python-sdk/blob/master/anki_vector/messaging/messages.proto) · [Anki playback implementation](https://github.com/anki/vector-python-sdk/blob/master/anki_vector/audio.py)
- Vector firmware source has a raw recording job. It interleaves four synchronous mic channels and writes a four-channel WAV at the incoming sample rate; its processed WAV writer uses one channel. The raw collection occurs before processed-audio handling. Availability of the recording job to an ordinary client on this stock unit and simultaneous speaker/raw-mic operation still require measurement. [Recording job](https://github.com/kercre123/victor/blob/snowboy/animProcess/src/cozmoAnim/micData/micDataSystem.cpp) · [Channel layout and WAV export](https://github.com/kercre123/victor/blob/snowboy/animProcess/src/cozmoAnim/micData/micDataInfo.cpp) · [Mic pipeline](https://github.com/kercre123/victor/blob/snowboy/animProcess/src/cozmoAnim/micData/micDataProcessor.cpp)
- Susurrus supplies audible-probe scheduling, carrier sense and quiet slots. Its real ggwave transport uses 48 kHz samples and gates RX during TX plus a 350 ms guard. On the tested Dell soundbar, echo cancellation corrupted self-decodes. That measurement applies to the Dell. A wall at 40 cm returns sound in about 2.3 ms, so the ranging receiver must capture throughout its own probe. A Susurrus peer could announce a quiet slot or witness the emitted chirp; peer reply time includes modem and scheduling latency.

## Offline control experiment

Run `python diagnose_vector_echo.py` in the simulator environment. A 4 ms, 2–6 kHz audible sweep is sampled on the raw mic clock. A matched filter detects the direct arrival separately on each channel, cancelling common playback latency. Add the known direct distance to the measured excess path length, then fit a reflected virtual speaker position for a single vertical wall. The source-to-wall range is half the distance from the real speaker to that virtual source. The fit uses an **assumed** 28 × 22 mm rectangular array and an offset speaker. Recording a ruler or CAD geometry and establishing the true channel order are prerequisites for bearing on the robot.

The seeded test varies playback onset, enclosure-reflection strength and independent noise. It uses one reflection at 3.5% of direct amplitude, a 0.8 ms enclosure return, azimuths −45°, 0° and +45°, and no room multipath or clipping. Echo amplitude is fixed across distances, so these figures test timing and geometry under the specified signal conditions. The threshold is the largest four-channel detection score in 100 empty calibration scenes.

| Independent test | Detections | Median absolute range / bearing error among detections |
| --- | ---: | ---: |
| No wall | 3/100 false positives | — |
| 20 cm wall | 6/60, spurious peaks | 370 mm / 150° |
| 40 cm wall | 60/60 | 0.28 mm / 2.34° |
| 80 cm wall | 60/60 | 0.28 mm / 2.23° |

The 1.4 ms rejection window intentionally masks the 20 cm return. The sub-millimetre figures reflect interpolation of one isolated synthetic peak at known geometry. At 4 kHz bandwidth the nominal two-reflector range resolution is about 4.3 cm; resolving separate echoes is harder than locating one ideal peak. A far-field 28 mm aperture spans only about 1.3 raw samples of differential arrival at most along that baseline. Bearing is especially sensitive to channel skew and enclosure reflections.

## Decisive physical check

At fixed head angle with motors still, capture all four raw channels beginning before playback. First verify continuity, channel identity, clipping and a stable direct peak. Repeat with an open background, then move a rigid board from 40 to 60 to 80 cm at constant azimuth. The expected extra delay per 20 cm step is about 1.17 ms (18 raw samples) near normal incidence. A delayed peak must track the board and disappear on removal; a fixed peak implicates the enclosure or table. Only after range passes, hold range fixed and mirror the board left/right to verify reversal of inter-channel lag. Repeat at another head angle because the speaker moves relative to the backpack array. If only synchronized mono is available, the direct/echo delay can still be tested for range. A coarse alternative with mono is a robot-heading sweep of echo strength against a fixed wall, requiring its own calibration. No ranging accuracy or navigational capability has been measured on stock Vector.

The follow-up traced the firmware sound check, identified the writer's 16 kHz header, and prepared a five-take board experiment. [Firmware evidence and bench procedure](vector-board-test.md)
