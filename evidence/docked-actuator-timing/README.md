# unit 2 docked actuator timing

![Head and lift profiles for three robots and their frozen replays](profiles.png)

unit 2 remained on the workshop dock and reported charging at the end. The recorder issued only head and lift actions. It requested three cycles per joint. unit 2 moved its head between the first two head actions, including a return from +46° to approximately −6° after the first raise completed. That cycle had unmatched starting postures. The raw recording includes it. Two head cycles and all three lift cycles met the endpoint check.

Median time from host command to detected motion and median 10–90% travel time, in milliseconds. Each cell gives onset / travel time. Frozen values here replay unit 2. The CSV includes the separate replay for every robot and stroke.

| Full stroke | unit 2 docked | Frozen | unit 1 run A | unit 3 run O |
|---|---:|---:|---:|---:|
| Head −22° to +45° | 220 / 366 | 105 / 375 | 179 / 373 | 207 / 363 |
| Head +45° to −22° | 157 / 353 | 98 / 380 | 180 / 361 | 251 / 357 |
| Lift 32 to 92 mm | 189 / 327 | 129 / 344 | 223 / 318 | 232 / 318 |
| Lift 92 to 32 mm | 190 / 341 | 129 / 338 | 199 / 326 | 279 / 318 |

unit 2 starts 59–115 ms later than its matched replay. After onset the largest median 10–90% discrepancy is 27 ms on head lowering. Individual unit 2 head lowers took 328 and 378 ms. The repeats and sample cadence limit conclusions about small differences. unit 3 starts its head lower about 94 ms later than unit 2, yet their median traversal times differ by only 4 ms.

## Measurement

All recordings use SDK state delivery time on the recorder clock. For the head, subtract the precommand pitch gyro baseline and interpolate the first sustained directional 1°/s crossing. For the lift, use the midpoint of the bracket containing the first sustained displacement above 0.15 mm. The two successive readings rule rejects isolated spikes. unit 2 onset brackets are typically about 60 ms wide. Host dispatch, transport, controller response and SDK reporting all contribute to measured onset. The data do not isolate motor dead time.

Select alternating full endpoint commands and require the median positions before command and during 1.25–1.53 s after command to be within 5° or 5 mm of the two endpoints. Interpolate 10% and 90% crossings of each measured excursion. Each overlay shifts only time and retains absolute angle or height.

Replay uses the unchanged actuator parameters at commit 69260a7, including the 105 ms command delay, measured initial joint positions, and each recording's sample times. The simulator's standard level-table replay supplies the reference for docked unit 2. Its generation was unverified, so the reference uses the base Vector 1 profile. unit 1 is a refurbished Vector 1. unit 3 is a Vector 2 with a printed lift and uses the Vector 2 profile. Both archived bots performed their sweeps off dock. These runs cannot independently identify a dock, generation or lift-hardware effect.

Run `MPLBACKEND=Agg python evidence/docked-actuator-timing/analyze.py` from the repository root with NumPy, Matplotlib and MuJoCo installed (analysis used MuJoCo 4.0.1). Inputs are the docked SDK log, the actuator excerpt from unit 3 run O before the wheelie phase, and the repository's unit 1 run A. See [per-move measurements](moves.csv).

## Three docked units

Two fresh docked captures from units 1 and 3 repeat the same command endpoints and analysis gates, with 1.45 s between action completion and the next command. The workshop's unit 2 was unreachable and absent from its access point's station list. Its earlier docked session supplies the third comparison. Units 1 and 3 both reported on the charger platform at the end. These sweeps issued no wheel commands.

Median physical 10–90% durations in ms (head up/down, lift up/down) were 366/348 and 305/336 for unit 1, 366/353 and 327/341 for archived unit 2, and 374/361 and 266/324 for unit 3. For unit 3 the frozen matched lift raise took 345 ms. Median receipt-clock onset on that stroke was 255 ms measured versus 121 ms simulated. The model starts early and traverses the stroke slowly on that unit, so a single duration measured from command dispatch hides both discrepancies. Most onset brackets span about one 60 ms state interval. The 21 accepted fresh strokes and 10 archived strokes are reported individually with the endpoint gates used above. No cause is assigned to generation, printed lift, or battery condition. [Overlay](repeat/profiles.png) · [per-stroke results](repeat/moves.csv) · [unit 1 raw](repeat/unit1.jsonl) · [unit 3 raw](repeat/unit3.jsonl) · [analysis](repeat/compare.py).
