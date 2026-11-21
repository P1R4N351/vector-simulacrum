# Vector hardware figures found online, 2026-11-15

Gathered by a lane for the Vector simulacrum. Every figure in the
"read and checked" sections was read by me from the fetched file; the SHA-256
prefix is of the bytes I fetched. Nothing here is measured on a robot.

## A. Released firmware (github.com/kercre123/victor, branch snowboy)

Raw base: https://raw.githubusercontent.com/kercre123/victor/snowboy/

### robot/hal/src/hal_motors.cpp (sha256 f87db43aff269b2d) — NEW source

    62:    ((       0.96 * 29.0 * 0.25 * 3.14159265359) / 172.3), //Left Tread mm (Magic 96% corrective fudge factor, 29mm wheel diameter)
    63:    ((-1.0 * 0.96 * 29.0 * 0.25 * 3.14159265359) / 172.3), //Right Tread mm
    64:    (0.25 * 3.14159265359) / 149.7,     //Lift radians
    65:    (0.25 * 3.14159265359) / 366.211,   //Head radians
    54:  static const f32 HAL_SEC_PER_TICK = (1.0 / 256) / 48000000;
    58:  const u32 ZERO_SPEED_ENCODER_TIC_THRESH = 5;
    150:    return (countsPerTick / HAL_SEC_PER_TICK) * HAL_MOTOR_POSITION_SCALE[m];

Computed from lines 62 to 65 (executed):
- tread: 0.12690 mm per encoder count, 7.880 counts per mm
- without the 0.96 factor: 0.13219 mm per count
- the firmware's odometry uses an effective wheel diameter of 27.84 mm (29 x 0.96)
- lift: 0.005246 rad per count (0.3006 deg)
- head: 0.002145 rad per count (0.1229 deg)

Not established: what 172.3, 149.7 and 366.211 are. They are not labelled.
Do not record them as gear ratios. (If the tread encoder gave 4 counts per
motor turn they would be gear ratios and the motor would turn about 25,000 rpm
at 220 mm/s, but the manual says the tread encoders are single-channel, so the
count per motor turn is unknown.)

### robot/supervisor/src/wheelController.cpp (sha256 407182f95afb3ea1) — NEW source (the table cites wheelController.h only)

    26:      const float DEFAULT_WHEEL_KP = 0.0005f;
    27:      const float DEFAULT_WHEEL_KI = 0.00005f;
    35:      f32 MAX_ERROR_SUM_LEFT = 5000.f;
    96-106:
    #     ifdef SIMULATOR
          f32 out_ol = x * 0.004f;
    #     else
          // Piecewise linear
          f32 out_ol = 0;
          if (x > 30) {
            out_ol = (float)(0.00313 * (double)x + 0.057);  // EP2
          } else {
            // power = speed  * 0.2 power /  30 mm/s
            out_ol = 0.00666667f * x;
          }
    #     endif
    112:      f32 out_corr = ( (Kp_ * error) + (error_sum * Ki_) );

So the real tread control is: motor power = feedforward(desired speed) +
Kp x speed error + Ki x summed error, with power on a -1..1 scale. Computed:
power 1.0 corresponds to 301 mm/s on the feedforward line; 220 mm/s to 0.746;
160 mm/s to 0.558. The two branches do not meet at 30 mm/s (0.151 against
0.200). This line is the firmware authors' own fit of power against tread
speed for the robot driving on a surface; it is the only published relation
between motor drive and speed.

### robot/supervisor/src/headController.cpp (sha256 cae6261c197b4945), hardware branch

    51:      f32 Kp_ = 4.f;  // proportional control constant
    52:      f32 Kd_ = 4000.f;  // derivative control constant
    53:      f32 Ki_ = 0.03f; // integral control constant
    54:      f32 MAX_ERROR_SUM = 15.f;
    121:      const f32 BRACING_POWER = -0.65;

### robot/supervisor/src/liftController.cpp (sha256 3f1912b32d54a2f4), hardware branch

    83:        f32 Kp_ = 3.f;     // proportional control constant
    84:        f32 Kd_ = 3000.f;  // derivative gain
    85:        f32 Ki_ = 0.1f;    // integral control constant
    87:        f32 MAX_ERROR_SUM = 5.f;
    96:        const f32 MAX_POWER_IN_POSITION_WHILE_CARRYING = 0.24f;
    100:        const f32 MAX_POWER_IN_POSITION = 0.1f;

These gains are in motor power per radian, not torque. They fix the shape of
the head and lift control, not its strength.

### robot/include/anki/cozmo/shared/cozmoConfig.h (sha256 50a02ad3004b5d6c) — already a source; lines worth a second look

    295:  const f32 MAX_WHEEL_ACCEL_MMPS2 = 10000.f;  // TODO: Actually measure this!
    197:  const u16 CLIFF_SENSOR_THRESHOLD_MAX = 180;
    199:  const u16 CLIFF_SENSOR_THRESHOLD_MAX = 40;
    201:  const u16 CLIFF_SENSOR_THRESHOLD_MIN = 15;
    205:  const u16 CLIFF_DETECT_HYSTERESIS = 30;
    310:  // Ideally, speed, in radians, should be (MAX_WHEEL_SPEED_MMPS / WHEEL_DIST_HALF_MM), but tread slip makes this not true.

### robot/syscon/src/opto.cpp (sha256 0cf1410a01cfc2c3)

    228:  uint32_t const MinTimingBudget = 20000;
    388:  // set final range signal rate limit to 0.25 MCPS (million counts per second)
    359:    writeReg(i, DROP_SENSOR_ADDRESS, PS_LED, 6 | (5 << 4));
    360:    writeReg(i, DROP_SENSOR_ADDRESS, PS_PULSES, 8);

The file uses VL53L0X register names, which confirms the range sensor part.

## B. Vector Technical Reference Manual (the copy already in use), lines of the extracted text

- 2040-2043: the 4 shaft encoders are photo-interrupters with a slotted disc
  on the motor shaft. The two tread motors use a Sharp GP1S092HCPIF
  (single channel). The lift and head motors use a dual-channel one.
- 2370-2376: tread encoders have a single emitter and detector.
- 1508: battery is single-cell 3.7 V 320 mAh lithium-ion polymer.
- 2052, 2340: ST Microelectronics VL53L0x time of flight sensor.
- The manual does not name the IMU part (0 lines match BMI160).

## C. Part datasheets

### Bosch BMI160, BST-BMI160-DS000-09, revision 1.0, November 2020 (sha256 0c3dca28517322bd)
https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf

- gyroscope output noise density: 0.007 deg/s per root Hz (at 10 Hz)
- gyroscope RMS noise: 0.07 deg/s (filter 74.6 Hz, ODR 200 Hz)
- gyroscope zero-rate offset: +/-3 deg/s; change over temperature 0.05 deg/s/K
- accelerometer output noise density: 180 micro-g per root Hz typical, 300 maximum
- accelerometer RMS noise: 1.8 mg (filter 80 Hz, ODR 200 Hz)
- accelerometer zero-g offset: +/-40 mg soldered, +/-150 mg over life; +/-1.0 mg/K

That the robot's IMU is a BMI160 rests on one community page only:
https://os-vector.github.io/vector-docs/5.-Hardware/index.html (sha256
ca576bd8e457c650), which says "IMU: BMI160". The manual and
robot/hal/src/hal_imu.cpp (sha256 4301a76e07cd85cf) do not name the part.

### ST VL53L0X, DocID029104 Rev 2, April 2018 (sha256 442686690c305410)
Read from a distributor's mirror, https://www.pololu.com/file/0J1187/VL53L0X.pdf
(st.com timed out).

- field of view: 25 degrees
- ranging standard deviation, indoor, 33 ms budget: 4% at 120 cm on a white
  (88%) target, 7% at 70 cm on a grey (17%) target; 3% and 6% at 66 ms
- maximum range, indoor, 33 ms budget: white typical 200 cm+ (long range
  profile), minimum 120 cm; grey typical 80 cm, minimum 70 cm
- default profile: 30 ms budget, 1.2 m; offset correction done at 10 cm

## D. Community hardware page (read, sha256 ca576bd8e457c650)

- "Cliff sensors: 4 Lite-On proximity sensors" (no part number)
- "Battery: FullRiver 320MaH"
- "The motors are a standard size, but Anki had a custom winding"
- "Each motor has an encoder, it is unknown where these come from"

## E. Reported by a fetch helper, not read by me

- Fictiv teardown, https://www.fictiv.com/teardowns/anki-vector-robot-teardown:
  "a 45-tooth internal gear that is used to drive the robot" inside the drive
  wheel; gearbox gears have "custom numbers of gear teeth" (no counts given).

## F. Looked for and not found

- Mass and outer dimensions. No page I or a helper could read states them.
  Retail listings were summarised by a search engine as 158.76 g (which is
  exactly 5.6 oz, so one origin) and 9.98 x 5.99 x 6.93 cm, but those pages
  returned 403 and whether the figures are the robot or the boxed product is
  unknown. Wikipedia, Anki's successor site, MacRumors, Wevolver and the
  RobotShop product sheet give neither.
- Gear ratios, gear tooth counts beyond the one above, gear stages.
- Motor part number, voltage, stall torque, no-load speed, current limits.
- Encoder slots per motor turn.
- Cliff sensor part number and its distance response.
- Centre of mass, part masses, tread friction: not published anywhere found.
