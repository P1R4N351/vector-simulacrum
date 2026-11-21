## Free-cube rescue with rubber corner pads

The movie follows one continuous MuJoCo trajectory per side, from cube presentation to withdrawal. A modeled offer pushes the free cube toward the patient at 0.28 N from 0.32 to 0.80 s. After release, only contacts act on the cube. Both forks can touch an adjacent face. The 50 g cube uses a free joint with eight 8 mm rubber corner regions standing 0.15 mm proud of the recessed shell. Pad size, projection, compliance, mass distribution and friction are modeling assumptions. The shell and pad sliding coefficients in the movie are 0.5 and 0.9.

| Event (s) | Right-side start | Left-side start |
|---|---:|---:|
| Patient fork first touches cube | 0.540 | 0.540 |
| Rescuer fork first touches cube | 1.971 | 1.975 |
| First robot-to-robot contact | 2.000 | 2.003 |
| Cube center has shifted >4 mm in patient's fork frame, relative to just before approach | 2.02 | 2.03 |
| Last patient fork contact with cube | 2.121 | 2.124 |
| Last direct robot contact | 2.959 | 2.888 |

Both patients settled upright after rescuer withdrawal. Excluding direct collisions between robots while preserving collisions with the cube left both patients on their sides (final tilt 90.18° and 90.15°). This mirrored maneuver depends on direct robot contact. It does not establish that the cube alone can transmit the righting torque. The contact markers show instantaneous contact rather than grip force.

Sensitivity to assumed pad sliding friction with shell friction held at 0.5: 0.6 and 0.9 righted both, while 1.2 left both sideways. These are exploratory selected trajectories. The physical fork fit, support surface, and rubber coefficient have yet to be validated.

To reproduce: unpack replay.zip and run python replay.py in that directory with mujoco 4.0.1 and numpy. The two saved initial states precede engagement. The control run uses the same initial states and controller with cross-robot collision pairs excluded. CSV output records contact at 1 ms. Film displays half speed and samples at lower cadence.
