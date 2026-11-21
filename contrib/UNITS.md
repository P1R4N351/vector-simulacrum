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

`fit_recording.py` reads every `contrib/units/<handle>/evidence/*.jsonl` as
if it were a recording under `evidence/`. Values that are per-unit in the
parameter table (see the README) widen or narrow their reported spread as
units accumulate. Values that are shared across the fleet (cube mass,
table dimensions) do not depend on which unit recorded; a contribution
still exercises the model against another real surface and another robot.

Per-unit values the current table lists:

- `com_x_mm`, `com_z_mm`, `com_y_mm`: centre of mass along the three axes.
- `wheel_bias_pct`: wheel-speed asymmetry.
- `backpack_lens_mm`: camera position on the backpack.
- `weight_g`: whole-robot mass with the lift empty.
- `accel_x_scale`: accelerometer x-axis scale.
- `lift_motor_torque_nm` and `lift_max_g`: lift strength.

Anything measured in the kit's standard phases lands on one of these; the
fit's existing scripts handle the rest.
