# Reproducing the 3.10 mm tread inset

On Linux x86_64 with MuJoCo 3.11.0, replaying C7K8's four run E spins with the current model gives **3.104737083567137 mm** per end. That exactly reproduces the [archived fit](../imu-filter-20261120/fit_spins-output.json). On the same host with MuJoCo 4.0.1 the fit gives **3.242032 mm**. The measurements and 48 mm physical axle spacing were the same in both runs. The crossing interpolates between integer-millimetre grid points and does not establish submillimetre physical accuracy.

| Build | C7K8 fit (mm) |
|---|---:|
| Linux x86_64, 3.11.0 | 3.105 |
| macOS arm64, 3.11.0 | 2.940 |
| Linux x86_64, 4.0.1 | 3.242 |
| macOS arm64, 4.0.1 | 3.242 |

Repeating the two macOS runs with the original model package from commit 9b8f606 gave the same crossings as the current package to the precision shown. The original recording differs only in a free-text note. These comparisons isolate a solver-version effect within each platform. The between-platform comparison also changes the operating system and native build, so it cannot isolate CPU architecture.

![P2Y8 steady yaw and turn center versus inset and native build](solver-transfer.png)

On MuJoCo 4.0.1, transferring 3.10 and 3.242 mm to P2Y8's four camera spins yields respectively **6.79% and 2.19%** RMS relative errors in steady head-gyro Z. The mean modeled pivot moves from −22.76 to −24.00 mm, versus −27.30 mm from the camera. At 3.75 mm the yaw error is 0.23% but the pivot moves farther forward to −21.96 mm. P2Y8 is held out for the C7K8 fit; it was used to select the separate 3.75 mm candidate. The historical fit uses head-gyro Z. The [recorded-pose experiment](../recorded-pose-inset/README.md) uses chassis-axis yaw and its own contact implementation.

The default calibration remains 3.10 mm. The regression checks the stored calibration against its archived fit, reproduces the crossing on the reference build, and checks the live fit against the documented range on each build. The independent rate and pivot regressions remain in place.

From the current repository root, run `PYTHONPATH=. python evidence/inset-portability/compare.py --model-label current` with the desired MuJoCo version. To replay the older model, point PYTHONPATH at an export of commit 9b8f606 containing simulacrum and the two parameter tables. The CSVs contain the four per-spin results at each inset. Run `python evidence/inset-portability/plot.py` to redraw the figure.
