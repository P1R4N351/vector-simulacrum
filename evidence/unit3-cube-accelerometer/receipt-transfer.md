### Cube receiver cadence on later physical repeats

The Vector 2.0 simulator emits 100 consecutive 20-byte notifications in three seconds, each 30 ms apart. At that interval, counting the packets received by the physical robot underestimates the elapsed duration by 16.96–17.66% on runs A, E and F. These later runs contain 4,482 notifications. Runs B, C and D in this series have no decoded accelerometer packets in the published radio logs.

Freeze three quantities on the earlier resting capture: 29.58 ms mean ordinary interval, 262.78 ms mean long interval and 33.76 packets mean spacing between long gaps. Start the periodic prediction at packet zero in every recording. This predicts the total received duration to within 0.71% on all three later runs. Maximum individual timeline errors remain 0.31–0.42 s. The median-based predictor gives a smaller maximum error on F. These results validate an elapsed-duration estimate for the recorded receiver on this setup. They cannot establish whether missing samples were generated, retained or lost. Do not retime physical impacts or change the simulator chip clock using this fit.

[Receiver intervals and prediction errors](receipt-transfer.png) · [Held-out scores](receipt-transfer.csv). Run receipt_transfer.py from this directory with the simulator dependencies installed.
