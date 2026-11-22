### Cube receiver cadence on later physical repeats

The Vector 2.0 simulator emits 100 consecutive 20-byte notifications in three seconds, each 30 ms apart. At that interval, counting the packets received by the physical robot underestimates the elapsed duration by 16.96–17.66% on runs A, E and F. These later runs contain 4,482 notifications. Runs B, C and D in this series have no decoded accelerometer packets in the published radio logs.

Freeze three quantities on the earlier resting capture: 29.58 ms mean ordinary interval, 262.78 ms mean long interval and 33.76 packets mean spacing between long gaps. Start the periodic prediction at packet zero in every recording. This predicts the total received duration to within 0.71% on all three later runs. Maximum individual timeline errors remain 0.31–0.42 s. The median-based predictor gives a smaller maximum error on F. These results validate an elapsed-duration estimate for the recorded receiver on this setup. They cannot establish whether missing samples were generated, retained or lost. Do not retime physical impacts or change the simulator chip clock using this fit.

[Receiver intervals and prediction errors](receipt-transfer.png) · [Held-out scores](receipt-transfer.csv). Run receipt_transfer.py from this directory with the simulator dependencies installed.

### Predicting receiver gaps from elapsed time

On the resting run, fit the time of the first accelerometer receipt after each gap longer than 150 ms against gap ordinal. The resulting period is 1.22746 s and the median gap is 260 ms. In each test run, use only the first observed restart as a phase anchor and predict all subsequent windows on the robot's uptime clock. Across five recordings the largest restart errors range from 13 to 86 ms. Total window intersection divided by union ranges from 0.771 to 0.950. The longest held-out run has the lowest overlap (E), and A has the largest single restart error. This predicts recurring blind windows when at least one restart is available. It does not reconstruct samples inside them.

In Bluetooth message order, only 69 of 178 long gaps in these five captures contain a battery-voltage notification. Battery reports alone cannot mark every pause. The daemon monitor timestamps receipt on a 10 ms uptime clock. This experiment identifies a receiver-time pattern without locating its cause in the firmware, radio or receiver.

[Predicted versus observed windows](receiver-phase.png) · [Per-recording scores](receiver-phase.csv). Run receiver_phase.py to reproduce.

### Do counter changes survive the receipt gaps?

Across 14 nonempty wheelstand-series captures, 555 long intervals (>150 ms) account for 145.52 s and 18,322 ordinary intervals account for 538.58 s. The transmitted eight-bit tap count advances once at 21 packet boundaries. Two follow long gaps, both in R5 (260 and 270 ms). Both returning packets contain an internal Z step above the firmware triggering threshold (3,996 and 5,724 raw counts against 2,880). R5 also contains SDK tap timestamps 27 ms before and 8 ms after the respective post-gap robot receipt. These offsets do not locate the physical impact inside the receipt interval.

Ten of the 19 other counter changes have no above-threshold Z step between the three slots of the new packet. The detector uses chip FIFO frames. The retained packet has just three tick samples, and the triggering transition can also precede the new packet. An analysis that infers taps solely by thresholding the two internal differences would miss those ten. This dataset provides no example of a post-gap tap count change accompanied by an entirely subthreshold returning packet. It cannot establish the rate of unseen physical impacts.

[Counter transitions](tap-gap.png) · [Per-event Z steps](tap-gap-events.csv) · [Intervals per recording](tap-gap-exposure.csv). Run tap_gap_audit.py.

### Packet listener versus SDK callbacks

We replayed 18,891 physical packets from 14 wheelstand-series recordings through the simulator's TapListener, retaining robot uptime and host receipt clocks separately. The listener emitted 21 events. All 20 captured SDK tap payload stamps matched one-to-one to listener events within 41 ms (100 ms assignment limit). The extra R10 counter change at uptime 8594.01 s had no matching SDK payload. The next counter change 150 ms later did match. Another 150 ms pair earlier in R10 generated two SDK events, so a simple 150 ms grouping rule would discard a real callback.

On matched taps, using the previously estimated 145.906 ms host delivery delay reduced median absolute callback-time error from 149.2 to 11.9 ms. The largest error after this correction was 171.3 ms. R10’s observed delivery delays ranged from 38.6 to 317.2 ms. The delay is a useful typical estimate on this setup. It is too variable to locate a particular impact. This replay supplies the *physical* packets, so it validates packet interpretation separately from predicting which contacts generate taps.

[Two clocks for each matched event](sdk-tap-transfer.png) · [All matches and extra listener event](sdk-tap-transfer.csv) · [Per-recording coverage](sdk-tap-coverage.csv). Run sdk_tap_transfer.py.

### Movement start and stop on the same physical packets

Passing each physical packet's three slots through the simulator's movement listener reproduced all five captured SDK movement transitions within 34 ms on the robot uptime clock. R10 moved for 660 ms in replay versus 640 ms by SDK timestamps. R11 moved for 350 ms versus 306 ms. R8's packet capture ended 207 ms after its SDK start, with no recorded stop. Use the individual transition rows for scoring; starts and stops can each receive different SDK clock offsets.

The listener also produced two starts and two stops in R5 without SDK movement callbacks. In host time, the first episode (26.800–29.252 s) was inside the pickup action; the second (32.844–38.343 s) ran from reversing with the cube into placement. All four occur during manipulation. This is consistent with the stock engine's separate carry/docking broadcast rules. Exact internal suppression state was not logged. The other ten nonempty captures had neither listener nor SDK movement transitions. Input packets came from real hardware, so no contact parameters were changed.

[Listener score and SDK events](sdk-move-transfer.png) · [Per-run and per-transition results](sdk-move-transfer.csv). Run sdk_move_transfer.py.

### Carry state explains which movement transitions reached the SDK

The four R5 candidates without an SDK callback were bracketed by robot states with IS_CARRYING_BLOCK (0x02) on. The five recorded SDK transitions in the wheelstand series all had the bit off. In a separate physical repeat (A), both movement transitions had the bit off and both reached the SDK. The broader IS_PICKING_OR_PLACING bit (0x04) would incorrectly suppress the genuine R8 movement start. The archived runs here each use one cube. Firmware checks the actual object ID for both carrying and docking, which a chassis-wide status bit cannot establish in a multi-cube scene.

The simulator now exposes movement.broadcast_events on the cube alongside movement.events. The former records eligible broadcasts at packet time. The latter retains every listener transition. VectorSim marks the cube mounted on the lift as handled. A caller with a dynamic carry/docking state can set cube.handling_cube when that particular cube is acquired or released. The packet filter and movement score continue to evolve while handled. Releasing a cube does not synthesize a movement transition. SDK transport delay is separate.

[Four candidate rules](movement-status.png) · [Bracketing states and SDK matches](movement-status.csv). Run movement_status.py to replay both collections and verify emitted events against the SDK.

### Independent pickup and release check

On the earlier pickup recording, passing the last reported carry ID before each packet into the new movement broadcast filter suppresses the middle two of four listener transitions. The first start at 3.710 s still comes through before acquisition, producing one extra event. On release the simulated listener broadcasts a stop at 12.645 s. The SDK delivered its sole callback 20.7 ms later, while its payload was stamped 21 ms before the radio monitor receipt on the robot clock. All four event times have the same reported carry ID on either side of the packet.

Stock firmware also checks whether this particular cube is the docking target. As a retrospective sensitivity check, marking it as handled while the pickup command is active suppresses the early start and reproduces the single delivered stop. Command lifetime is only a proxy for the unlogged docking target. Extending suppression through the whole placement command would incorrectly remove the real stop, which preceded command completion by 0.39 s. The earlier seven-event carry-only result therefore does not generalize to the pre-acquisition part of pickup.

[Physical episode](release-broadcast.png) · [Carry-only and pickup-target predictions](release-broadcast.csv). Run release_broadcast.py.
