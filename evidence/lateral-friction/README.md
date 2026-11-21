# Does lateral tread friction explain unit 3’s drift?

Both 42 mm contact models preferred lateral friction 0.8875 at spot B with the B inclination imposed. It improved the angular-speed match at held-out A, but **both models predicted the final position worse**. The data do not support changing the simulator default.

| Spot | Contact shape | Paired displacement RMSE, mm | Full gyro RMSE, °/s | Mean absolute steady bias, °/s |
|---|---|---:|---:|---:|
| B, selection | Cylinder | 11.75 → 10.74 | 17.98 → 17.15 | 8.56 → 2.31 |
| B, selection | Sphere | 11.05 → 9.65 | 14.77 → 13.86 | 7.56 → 5.29 |
| A, held out | Cylinder | 16.96 → 19.57 | 18.03 → 17.00 | 7.49 → 4.78 |
| A, held out | Sphere | 11.69 → 12.65 | 14.90 → 13.50 | 8.67 → 3.98 |

Each arrow compares 0.90 with 0.8875. At A the cylinder actually improved individual-turn displacement RMSE (12.79 → 10.82 mm). Its two turn errors accumulated differently, leaving a worse final position. Projecting paired error onto the imposed downhill direction also exposes the heading dependence. For the cylinder at B the 0° excess increased from 7.4 to 13.9 mm while the 180° excess fell from 9.8 to 3.6 mm. One coefficient can change skid-steer yaw resistance, but the observed translation errors depend on where and how the robot starts. Two starting headings per spot limit how precisely we can distinguish tread seating, surface variation and camera errors.

## Head posture and the level control

For each stationary accelerometer sample we used its head encoder angle to transform head X/Z into body-forward acceleration before fitting a heading-dependent vector and a body-fixed offset. Ignoring head pitch gives apparent inclinations of 0.949° at B and 0.988° at A. Applying it gives 0.982° and 0.911°. The B fit predicted horizontal acceleration at four new post-spin resting poses at A with 4.34 mg vector RMS error. A level prediction with a constant body offset fitted at B gave 15.62 mg. At four post-spin B poses these errors were 4.75 and 15.23 mg respectively. Rest windows span 0.45–1.25 s after stop with zero reported wheel speeds and gyro norm below 0.05 rad/s.

This identifies a repeatable inclination in the robot’s resting measurements. An independent measurement of the bare shelf would be needed to assign it to the shelf plane. On the level replay, neither contact shape had a lateral coefficient that improved the B displacement score within the gyro tolerances. At baseline friction, held-out A paired displacement errors were 27.93 mm (cylinder) and 27.18 mm (sphere).

The moving comparison uses head-frame gyro Z on both sides. Recorded head angles spanned −1.36° to −0.62° across the eight spins. Seven had constant reported head position and one varied by 0.25°. The firmware’s yaw projection, dividing Z by cos(head pitch), changes these samples by at most 0.061°/s. Head posture matters for gravity subtraction even when this yaw projection is small. [Firmware IMU reporting](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/supervisor/src/messages.cpp#L93-L100)

## Replay design

Measured wheel speeds drive each spin. The 42 mm effective span, physical axle positions and longitudinal friction (0.90) are fixed. The replay-only contact hook aligns the first friction tangent with chassis forward projected onto the contact plane. The worker checks the solver frame and both sliding coefficients during replay. Setting both coefficients to 0.90 reproduces the historical drift scores to within 0.000001 mm. The historical control is encoded as −1 in the CSVs. Each gravity condition selects a candidate independently on B and only then evaluates A. Selection minimizes paired displacement RMSE among coefficients improving the historical and aligned isotropic controls, allowing at most 1°/s degradation in full gyro RMSE and mean absolute steady bias. When none qualify, the saved candidate is diagnostic only.

[Camera endpoint overlays](../physics-plots/lateral-friction-transfer.png) · [Gyro overlays](../physics-plots/lateral-friction-gyro.png) · [B sweep and level control](../physics-plots/lateral-friction-sweep.png)

Reproduce from the repository root with MuJoCo 4.0.1, the project’s plotting dependencies and a C compiler:

```sh
python replay_lateral_friction.py
python replay_lateral_friction.py --level
python check_friction_imu.py
python plot_lateral_friction.py
```
