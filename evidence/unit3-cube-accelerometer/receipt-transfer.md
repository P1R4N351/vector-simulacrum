
### Does radio-monitor receipt jitter explain the late tap callbacks?

We predicted host callback receipt for each matched tap using the previously estimated 145.906 ms delay. A second predictor replaced the current packet's host receipt time with its robot uptime mapped onto the host clock by the median offset of packets from the preceding second. Both predictors use the same matched physical events and the same frozen delay. Neither estimates missing taps.

Across 20 wheelstand-series taps, mapping uptime changed median absolute error from 11.9 to 12.9 ms and maximum absolute error from 171.3 to 174.5 ms. At the two largest R10 errors, the current receipts differed from their mapped times by only −11.1 and +3.2 ms. Among three independent docked-grip taps the median improved from 13.4 to 9.8 ms. These recordings leave the engine dispatch and SDK delivery contributions unresolved. We kept the simulator's detector timestamps and the receiver timing estimate separate.

[Matched-event comparison](tap-receipt-clock.png) · [Per-event errors](tap-receipt-clock.csv) · [Reproduce](tap_receipt_clock.py)
