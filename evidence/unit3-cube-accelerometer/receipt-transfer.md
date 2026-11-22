### Cube receiver cadence on later physical repeats

The Vector 2.0 simulator emits 100 consecutive 20-byte notifications in three seconds, each 30 ms apart. At that interval, counting the packets received by the physical robot underestimates the elapsed duration by 16.96–17.66% on runs A, E and F. These later runs contain 4,482 notifications. Runs B, C and D in this series have no decoded accelerometer packets in the published radio logs.

Freeze three quantities on the earlier resting capture: 29.58 ms mean ordinary interval, 262.78 ms mean long interval and 33.76 packets mean spacing between long gaps. Start the periodic prediction at packet zero in every recording. This predicts the total received duration to within 0.71% on all three later runs. Maximum individual timeline errors remain 0.31–0.42 s. The median-based predictor gives a smaller maximum error on F. These results validate an elapsed-duration estimate for the recorded receiver on this setup. They cannot establish whether missing samples were generated, retained or lost. Do not retime physical impacts or change the simulator chip clock using this fit.

[Receiver intervals and prediction errors](receipt-transfer.png) · [Held-out scores](receipt-transfer.csv). Run receipt_transfer.py from this directory with the simulator dependencies installed.

### Predicting receiver gaps from elapsed time

On the resting run, fit the time of the first accelerometer receipt after each gap longer than 150 ms against gap ordinal. The resulting period is 1.22746 s and the median gap is 260 ms. In each test run, use only the first observed restart as a phase anchor and predict all subsequent windows on the robot's uptime clock. Across five recordings the largest restart errors range from 13 to 86 ms. Total window intersection divided by union ranges from 0.771 to 0.950. The longest held-out run has the lowest overlap (E), and A has the largest single restart error. This predicts recurring blind windows when at least one restart is available. It does not reconstruct samples inside them.

In Bluetooth message order, only 69 of 178 long gaps in these five captures contain a battery-voltage notification. Battery reports alone cannot mark every pause. The daemon monitor timestamps receipt on a 10 ms uptime clock. This experiment identifies a receiver-time pattern without locating its cause in the firmware, radio or receiver.

[Predicted versus observed windows](receiver-phase.png) · [Per-recording scores](receiver-phase.csv). Run receiver_phase.py to reproduce.
