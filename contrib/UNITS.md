# Contributed units

One row per contributed unit as they land. "Where it feeds" names the
part of the parameter table that spreads once more than one contributed
unit covers it.

| Handle | Hardware | Firmware | Phases recorded | Licence | Where it feeds |
|---|---|---|---|---|---|
| (none yet) | | | | | |

A submission adds one row above the empty row and leaves its own entry
above the previous one. The empty row stays at the bottom.

## How a unit feeds the fits

Validated recordings under `contrib/units/<handle>/evidence/*.jsonl` are
formatted identically to recordings under `evidence/`. Today `fit_recording.py`
does not scan `contrib/units/` automatically; accepted unit folders are
incorporated into per-unit refits and spread analyses once validated. Values
that are per-unit in the parameter table (`parameters.json` and
`parameters-vector2.json`) widen or narrow their reported spread as units
accumulate. Values that are shared across the fleet (cube dimensions, table
dimensions) do not depend on which unit recorded; a contribution still exercises
the model against another real surface and another robot.

Per-unit values in the current parameter tables:

- `com_x_mm`, `com_z_mm`: centre of mass fore-aft and vertical.
- `total_mass_g`: whole-robot mass with the lift empty.
- `accel_scale`, `accel_bias_mg`, `imu_pitch_offset_deg`, `gyro_bias_dps`: per-unit head IMU calibration.
- `lift_servo_torque_max_mNm`: lift motor strength.
- `tread_heat_loss_rate_per_s`, `tread_heat_tau_s`: per-tread sustained-load thermal derating.

Anything measured in the kit's standard phases lands on one of these or extends
held-out replay coverage across units.
