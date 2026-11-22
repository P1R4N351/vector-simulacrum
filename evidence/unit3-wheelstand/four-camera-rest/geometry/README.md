# Three-face check on the restored Arducam

We used the processed stationary image from the four-camera run. The outer glyph borders were modeled as centered 23 mm squares on perpendicular faces of a nominal 44 mm cube. The rounded printed corners were extrapolated by intersecting robust line fits to the middle 60% of each straight edge. Each fit used eight corners on two faces. All four corners on the remaining face were reserved for prediction.

At grayscale threshold 145, a square-pixel pinhole with centered principal point and estimated focal length had held-out corner RMS errors of 10.06 px (top), 10.35 px (left), and 9.11 px (right). Allowing the distance between opposite glyph planes to vary produced 10.41, 12.05 and 11.75 px respectively. The fitted distances ranged from 42.32 to 47.86 mm. A general projective camera with nominal cube geometry gave 2.74, 4.27 and 3.05 px. This last camera has eleven independent parameters estimated from just eight points on one cube pose. It is only a local diagnostic.

The image alone cannot distinguish lens distortion, principal point or pixel scaling from cube face recess, printed glyph spacing or contour localization. Fitting these corners does not justify resizing the cube or claiming a metric calibration across the shelf. The observed stationary subpixel tracking scatter addresses another question.

![Observed border intersections in green, nominal camera predictions in magenta, and free projective predictions in orange](face-transfer.jpg)

[All three edge thresholds and fit errors](face-transfer.csv) · [processed source crop](source.png) · [reproduce with Python, SciPy and OpenCV](check.py).

### Browser glyph dimensions

We measured the browser's actual optical model using the pinned SDK mesh and atlas, the browser's per-axis scaling to 44 mm, and its top-origin UV convention. We located the atlas border at grayscale 128 and mapped its edge midpoints to the mesh. All six glyph planes coincide with the outer face planes to numerical precision. Their apparent dimensions span 21.89–23.28 mm and their centers lie 0.48–1.19 mm off the face centers. One border midpoint falls just outside the UV panel and uses slight extrapolation. These measurements describe the SDK artwork. The physical cube still requires independent metric reference.

For the camera comparison we extrapolated straight border lines from the interior mapping and compared the three rectified physical symbol interiors to all 24 proper cube rotations. The two best mean correlation scores were 0.672 and 0.662. Orientation is ambiguous, so we scored both. With the general projective camera, nominal centered squares yielded held-out RMS of 2.74/4.27/3.05 px (top/left/right). The browser artwork yielded 4.10/6.29/5.95 px for the first orientation and 4.01/7.15/3.29 px for the second. Switching the calibration target to the SDK artwork worsens every face in both hypotheses. We retain nominal squares for physical camera reconstruction. No physical recess or offset has been inferred from the SDK.

[Six-face dimensions](sdk-glyphs.csv) · [per-face scores](sdk-projection.csv). To reproduce, install the optional SDK assets with the repository's web/tools/fetch_sdk_model.py, then run sdk_glyphs.py and sdk_projection.py. Only measurements and analysis code are included here.
