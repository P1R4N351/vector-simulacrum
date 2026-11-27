# Canonical dock idle creep fix

## Root cause
Two solver defaults allowed a robot parked at rest on `dock_pocket` (`37.6°` slope, `friction = 0.05`) and `dock_bed` to creep downhill (`3.131 mm` / `14.761 mm` over `15 s` / `70.7 s` on `vector-free-dock`, disconnecting at `67.725 s`; `4.823 mm` / `22.842 mm`, disconnecting at `43.675 s` on `vector-contact42-free-dock`):

1. Driven front wheel joints (`wheel_fl_joint`, `wheel_fr_joint`) set `frictionloss` without `solreffriction`, using MuJoCo's `0.02 s` constraint time constant rather than the firmware's `control_period_ms` (`5 ms`).
2. `dock_pocket` set `priority="2"` without `solref` (falling back to `0.02 s`) or `solimp="0.999 0.999 0.001"`, while `ramp` geoms (`dock_lip`, `dock_apron`, `dock_bed`) used soft default `solimp`.

Why three alternative prototypes were rejected:
- `evidence/dock-impedance/variant.mjs` (`pr331_both`) set `priority="3"` without `friction` or `solref`, silently overriding `dock_bed` friction (`tread_friction = 0.9` -> MuJoCo default `1.0`) and `solref` (`0.004 1` -> `0.02 1`).
- Setting `solimpfriction="0.999 0.999 0.001"` globally on front drive hinges or stiffening entry side guides (`dock_funnel_l/r`) broke non-dock and climb invariants: global `solimpfriction` shifted flat-table `spin_cw_220` yaw error past `0.02` (`0.02274`) and stopped the `9.15%` sub-breakaway motor packet in `web/test/wire-body.test.js` (`0 -> 0` ticks), while stiff side funnels deflected off-center backup climbs (`web/test/dock.test.js`, `y = +13.85 mm`).
- Setting driven-wheel `solreffriction` to `contact_time_constant_ms` (`4 ms`) placed the `robot.test.js` `drive_straight(400, 100)` action sequence on a 1-tick native-vs-WASM rounding edge (`3159` vs `3150`); matching `control_period_ms` (`5 ms`) restores bit-exact Python/WASM action clock parity (`3151 == 3151`, `±0` nm sensitivity) and keeps `spin_cw_220` error at `-0.01497` (vs baseline `-0.01639`).

## Canonical change
- `_wheel_xml`: emit `solreffriction="{fric_tc} 1"` (`fric_tc = control_period_ms * 0.001`) alongside `frictionloss` on driven front wheel hinges (`wheel_fl_joint`, `wheel_fr_joint`).
- `_dock_geom_attrs`: apply `solref="{tc} 1" solimp="0.999 0.999 0.001"` (`tc = contact_time_constant_ms * 0.001`) only to `dock_pocket` (preserving `priority="2" condim="3" friction="0.05 0.005 0.0001"`) and `ramp` geoms (`dock_lip`, `dock_apron`, `dock_bed`, setting `priority="2" condim="3"` and preserving `friction="{tread_friction} 0.005 0.0001"`). Side entry guides (`dock_funnel_l/r`, `dock_rail_l/r`, `dock_tower`) and `shell` geoms stay unchanged.

## Validation summary (MuJoCo 4.0.1)

### Idle hold (70.7 s rest followed by `60 mm/s` drive command)
| Scene | Variant | 15 s creep | 70.7 s creep | Mean shaft 70.7 s | Idle disconnect | Release after cmd |
|---|---|---:|---:|---:|---:|---:|
| `v1-free-dock` | Canonical prior | 3.131 mm | 14.761 mm | 7.931 mm | 67.725 s | 5.0 ms |
| `v1-free-dock` | Canonical fix (`5 ms` `solreffric` + support dock) | 0.456 mm | 2.128 mm | 2.032 mm | Never | 410.0 ms |
| `v1-contact42-dock` | Canonical prior | 4.823 mm | 22.842 mm | 8.782 mm | 43.675 s | 5.0 ms |
| `v1-contact42-dock` | Canonical fix (`5 ms` `solreffric` + support dock) | 0.543 mm | 2.394 mm | 2.244 mm | Never | 405.0 ms |
| `v2-free-dock` | Canonical prior | 3.131 mm | 14.761 mm | 7.931 mm | 67.725 s | 5.0 ms |
| `v2-free-dock` | Canonical fix (`5 ms` `solreffric` + support dock) | 0.456 mm | 2.128 mm | 2.032 mm | Never | 410.0 ms |

### W35 optical departure (`shaft` + `glyph-lip` seating)
- Canonical prior: `shift = 7.398 mm`, release `-42.0 ms` (inside `[-81, 0] ms`), `rearRMSE = 3.45 mm`, `frontRMSE = 2.49 mm`.
- Canonical fix: `shift = 7.500 mm`, release `-42.0 ms` (inside `[-81, 0] ms`), `rearRMSE = 3.44 mm`, `frontRMSE = 2.48 mm`.

### A–F shaft departures (`6 mm` seat shift)
- Canonical prior: A–C mean bracket miss `0.3 ms` (`0.94 mm` err), D–F `11.6 ms` (`0.81 mm` err).
- Canonical fix: A–C mean bracket miss `3.7 ms` (`0.92 mm` err), D–F `9.9 ms` (`0.95 mm` err, reflecting removal of pre-departure downhill solver creep rather than worsened departure tracking).
