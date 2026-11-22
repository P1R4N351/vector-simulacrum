# Hold-out roll approach with no measurable cube movement

A native undock and cube connection succeeded. The robot saw the cube at 79.2 mm and issued a single native roll. The SDK returned VISUAL_OBSERVATION_FAILED after 8.58 s. The recorder kept the cube connected through eight seconds of passive observation after motor stop. The robot completed native docking. Charger and quiet-mode status were verified after release.

From command through the end of passive observation, 1,569 acceleration samples arrived in 523 packets. The tap counter stayed at 3, with zero changes and zero SDK tap or movement reports. The cube remained +Y up. Mean gravity changed by 0.011° between the one-second pre-command window and the final two seconds of observation. That value describes agreement between sample means; true pose precision is lower. Accelerometry alone cannot exclude slow level sliding or yaw.

The rolling-tread replay from the preceding successful physical roll was frozen before this experiment. We supplied the new chassis and lift trace, observed horizontal cube center, and pre-command SDK cube quaternion corrected to the nearest flat resting face. Since the cube started on another face, orientation is now an explicit replay input. The physical accelerometer and SDK cube pose use different conventions; this comparison scores acceleration magnitude, tilt from the starting vertical and tap changes. No coefficients were fitted. With a 0.25 ms step the simulated tread never contacted the cube. Predicted peak tilt was less than 0.001°, with zero tap-counter changes. Peak packet magnitude after 0.5 s was 1.007 g simulated and 0.982 g measured. The run tests rejection of a false disturbance. It adds no positive flip or impact benchmark.

[Overhead clip](approach.mp4) · [Physical traces](post-stop.png) · [Held-out replay](holdout.png)

Run holdout.py with the printed fixture XML described in ../R8. Run poststop_metrics.py with this directory as its argument. Times in actions, state, cube and events are relative to the undock command; roll-poses uses seconds after the roll command.
