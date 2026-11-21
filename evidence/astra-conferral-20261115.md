# Conferral with an external review lane, 2026-11-15

Asked by a lane at its instruction ("confer with astra"), through the
seat service. One ask, one reply.

- Route: `copilot/gpt-6-astra`; model `gpt-6-astra`; 75.9 s; `failed_before`: [] (no fall-through to another provider).
- Prompt SHA-256 `6bef418e6f27ff3c19c3766f10a0fd487d178c6de70925195b2212f35358f05b`; answer SHA-256 `4a0cd78cfe736a194c1c3fd96369b077b140994e4340cd56b2cb57d2d6e2b09f`. Both were recomputed here and match.

The reply is another model's opinion. Nothing in it was adopted without being
checked by running something; the last section says what was checked and what
came of it.

## The ask

```
Conferral from a lane (instance a lane). Independent read wanted; disagree where you disagree. Reply in at most 500 words, numbered to match the questions, findings first.

CONTEXT. A MuJoCo model of a small tracked desk robot (Anki Vector: two treads, 46 mm tread spacing, 48 mm wheel base, about 160-170 g, head and lift servos). Goal: train a reinforcement-learning policy in simulation that transfers to the one real robot. Task: follow a commanded forward speed (0-120 mm/s) and turn rate; action = two wheel speeds every 60 ms; observation = reported wheel speeds (encoder, whole mm/s), gyro xyz, accelerometer xyz, one cliff bit, forward range, head angle, lift height, command, last action. Episodes on a table with open edges; a fall is -20; reward widths 40 mm/s and 30 deg/s. Every ranged parameter is redrawn per episode.

The parameter table has 138 entries. 74 come from the vendor's released firmware and its own Webots model, 17 from a reference manual, 9 are measured and 10 fitted from one 162 s recording of the real robot (SDK state at 60 ms: wheel speeds, gyro, accel, head, lift, cliff bit; no ground-truth pose), 28 are still assumed with a range. The assumed ones: total mass, head/lift/wheel masses, centre of mass x and z, tread friction (0.9, range 0.5-1.3), chassis friction, ground clearance, cliff sensor height and detection range, head and lift servo gains / torque limit / rotor inertia, head collision box, and solver settings (1 ms step, contact time constant) and table size. The tread drive itself is the firmware's own control law with a fitted motor (stall torque, no-load speed, friction share, rotor inertia; only the torque/inertia ratio is identified).

WHAT I AM DOING NOW. The owner asked: resolve the assumed values with real ones, in order of greatest impact on the simulation, for sim-to-hardware portability.

(a) Ranking method. For each assumed entry, set it to its low bound, then its high bound, everything else nominal, and measure against the nominal table: (i) closed loop: the training task itself with randomisation off, 24 paired episodes (same seeds), driven by a blind no-slip controller and by a weak exported policy (150 PPO iterations, not yet better than the blind controller); change in reward per step and falls. (ii) open loop with sensor noise off: mean change of true forward speed and turn rate in units of the reward widths over fixed wheel-command scripts; largest RMS change of any reported observation channel; outcomes of three table-edge runs. Order = largest absolute change in reward per step.

(b) Two findings from re-reading the recording.
1. The gyro and accelerometer sit in the head. Over 369 turning samples gyro_x/gyro_z = 0.9985*tan(head angle) - 0.0185. The head was 22.4 deg up in all six recorded spins, so my earlier figure "the robot turns at 0.46 of the no-slip rate" (from gyro z alone) was low: from gyro x and z together it is 0.44-0.53, mean 0.50 (n = 6 spins at 30/60/100 mm/s; clockwise faster than counter-clockwise in all three pairs). The model, which matched head-frame gyro z in replay only partly, turns at 0.44-0.46 of no-slip in truth. No fitted parameter currently sets that ratio; it emerges from four-wheel skid-steer contact with isotropic friction and the assumed centre of mass.
2. Accelerometer at rest: least squares over 26 still stretches on the (tilted, 1.4 deg) table, three clusters of head angle (22.4, about 7.5, about 0 deg), headings mostly near 180 deg with three near 0 and three near 124: unknowns table tilt x/y, pitch of the sensor frame, one offset per axis. Result: offsets -47.0, -7.7, +22.3 mg (formal s.e. 3.0, 0.6, 0.4), frame pitch -0.66 deg (s.e. 0.17), rms residual 17 mm/s^2; leaving out any one head cluster moves offsets by under 1.2 mg. Gravity fixed at 9806.65. I entered these as "fitted" with a +/-10 mg range and applied the pitch to the simulated sensor frame.

QUESTIONS.
1. Is ranking (a) a sound way to order the unknowns for transfer? What would it systematically miss or overrate (one-at-a-time bounds, a weak policy as the probe, author-chosen ranges, paired episodes that can flip chaotically)? Name a better cheap probe if there is one.
2. The 0.50 vs 0.45 turn-rate gap: should I fit a physical unknown to it (which: centre of mass x, lateral vs longitudinal tread friction, contact patch geometry), or is fitting one scalar with several candidate knobs worse than leaving it and randomising? What extra measurement would separate the candidates?
3. Is the accelerometer fit trustworthy as described? What could make offset x and frame pitch trade against each other despite the leave-one-cluster-out result, given the head clusters and headings are confounded (the 22 deg cluster faces the other way from most of the rest)?
4. For this task and observation set, which three unknowns would you measure by hand first (kitchen scale, balance on an edge, tilt board, ruler), and which assumed entries would you stop worrying about?
5. Anything in this plan you would not do.
```

## The reply, verbatim

**a reviewer-12c55c2**

1. **Useful screening, not a transfer-priority ranking.** Absolute reward change conflates correcting nominal error with causing damage. One-at-a-time endpoints miss interactions, thresholds, and compensating errors; chosen ranges largely determine rank. Weak policies reveal their own failure modes, not necessarily those of a trained policy. Paired seeds help, but trajectories diverge; report confidence intervals and falls separately—24 episodes poorly resolve rare failures. Normalize observation differences by meaningful channel scales. Cheap improvement: Morris-style elementary effects over several randomized backgrounds, using short straight/turn/reversal scripts plus edge approaches. Retain signed changes. Prioritize sensitivity × uncertainty-reduction achievable per measurement cost, not sensitivity alone.

2. **Do not identify a physical knob from this scalar.** The simulated ratio overlaps the observed range; compare matched speed/direction conditions and uncertainty across six spins, not 369 samples. Judge the discrepancy in deg/s relative to reward width. First verify sensor-to-body rotation, gyro bias, and measured—not merely commanded—wheel speeds. COM, friction anisotropy, and contact geometry remain confounded. Measure COM and contact footprint independently; add overhead-video yaw/translation during straight, in-place, and finite-radius motion in both directions at several speeds, with head fixed. Directional drag tests add friction evidence. Until then, randomize a calibrated effective turning discrepancy or plausible contact-model families; do not “measure” COM by forcing one yaw ratio.

3. **Plausible fit; stronger precision claims are not yet justified.** Near level, a small pitch change produces an almost constant x-gravity change, readily absorbed by x-offset; limited head-angle coverage weakly breaks that degeneracy. Confounded headings let table tilt participate. Head-angle zero error is also indistinguishable from mounting pitch without an independent reference. Leaving out clusters within the same confounded design does not establish identifiability. Inspect the Jacobian’s conditioning and joint parameter covariance; bootstrap independent still stretches. Recollect a crossed head-angle × heading grid. Treat ±10 mg as provisional, and preserve pitch/offset correlations rather than independently redrawing them.

4. **For this edge-exposed task, my provisional first three are:**
   - **Effective cliff-trigger margin relative to the supporting tread footprint:** ruler, slow restrained approaches, several headings/lighting conditions. This is more useful than separately guessing sensor height and range.
   - **Fore–aft COM:** balance test with head/lift in task postures; constrains contact loading and tipping.
   - **Total mass:** kitchen scale; cheap and anchors motor/contact dynamics.

   Directional traction is next; tilt-board breakaway measures static traction, not dynamic skid resistance. Defer individual link masses, servo internals, and detailed head collision geometry **if** head/lift remain stationary and those parts never contact anything. Revisit COM height if tipping sensitivity appears.

5. **I would stop independent redraws of physically coupled quantities.** Enforce consistent total/link masses, COM/inertias, identified torque/inertia combinations, and calibration covariance. Treat timestep and numerical contact settings as convergence checks, not physical unknowns to “resolve”; table size is an environment specification. Do not retune numerous parameters against this single recording without held-out validation. For real edge tests, use a catch surface or restraint that remains unloaded before a fall.

## What was checked, and what came of it

| Point | Checked by | Outcome |
|---|---|---|
| 1. The ranking is a screen, not a priority; a weak policy and paired episodes are noisy; report falls and signed changes separately | `rank_assumed.py --null 6`: six runs with the robot's mass changed by 0.1 to 0.6 percent, a change that should not matter | Confirmed. See the README's ranking section for the floor those runs set and which entries clear it. Signed changes and falls are printed per bound. Elementary effects over randomised backgrounds were not done |
| 2. Do not fit a physical value to the one turn-rate number; compare at matched speeds in deg/s | `measure.py`: the model's spin fraction at 30, 60 and 100 mm/s beside the recording's, and at four values of `com_x_mm` | Followed. The shortfall is 3.3, 8.6 and 12.0 deg/s. The fraction is not monotonic in `com_x_mm` (0.573, 0.443, 0.560, 0.741 at -30, -24, -19, -14 mm), so one number could not have chosen a value anyway. Nothing was fitted to it |
| 3. Pitch and x offset can trade; leave-one-cluster-out does not show identifiability; inspect covariance, bootstrap | `simulacrum/imu_fit.py`: correlation from the fit's covariance, bootstrap over the 26 still stretches, condition number with scaled columns | Confirmed. Correlation -0.976; bootstrap 95 percent: pitch -0.79 to -0.56 deg, x offset -48.9 to -45.1 mg; their sum (x on a level surface) -60.3 to -57.3 mg; scaled condition number 19. Both are in the fit's report and tests. A crossed head-angle by heading grid is added to the next recording. Training still draws the two independently |
| 4. Measure first: cliff trigger margin, fore-aft centre of mass, mass | Compared with the ranking | Agrees on the centre of mass. The cliff sensor height and range have no effect at all in this model, which supports measuring the margin itself instead. The README's hand list follows this order with the centre of mass first |
| 5. Solver settings are convergence checks; table size is a specification; no coupled redraws; no retuning without held-out data | Table reclassified | Seven entries moved from `assumed` to a new class `chosen`. Coupled redraws (masses with centre of mass, pitch with x offset) are not implemented and are listed as not done. No fitted tread value was changed |
