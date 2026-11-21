"""The IMU chip's own low-pass filter, run at every physics step..

Needs numpy, not MuJoCo.

The firmware sets the BMI160 to 200 Hz output with the accelerometer in
OSR2 and the gyro in OSR4, and reads the chip's filtered data
(IMUC:132-134 ACC_RATE_200HZ 0x19 and GYRO_RATE_200HZ_OVS_4X 0x09, written
at IMUC:241 and 244; IMUC:145 FIFO_FILTER_NO_DOWNSAMPLE 0x88, "filtered
data w/ no downsampling", IMUC:249). Nothing after the chip filters again
before the state message: MSGCPP:94-100 copies the latest raw accelerometer
sample and the gyro less its bias estimate (IMUCPP:950).

The datasheet gives each setting's -3 dB point and says that in OSR2 and
OSR4 "both stages of the digital filter are used" (BMI160DS:1075, 1153,
1160). It does not give either stage's shape. This module models the two
stages as one-pole sections in series and places their corners so that
the pair is -3 dB at the datasheet's figure. second_pole is the ratio of
the lower corner to the higher one: 0 is one section alone, 1 is two equal
sections. A stage of higher order than one is not modelled.

Each section is the matched one-pole y += a (u - y) with
a = 1 - exp(-2 pi f h), updated once per physics step h. The chip
samples its output at 200 Hz, which is the control tick, so the robot
reads the filter's state once per tick.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

SECTIONS = 2
CHANNELS = 6  # accelerometer x, y, z, then gyro x, y, z


def corners_hz(cutoff_hz: float, second_pole: float) -> Tuple[float, float]:
    """The two sections' corner frequencies whose series response is -3 dB at cutoff_hz.

    With x = (cutoff / lower corner)^2 and q = second_pole, -3 dB means
    (1 + x)(1 + q^2 x) = 2. Its positive root, written so that q = 0 needs
    no special case, is x = 2 / ((1 + q^2) + sqrt((1 + q^2)^2 + 4 q^2)).
    """
    assert cutoff_hz > 0.0 and math.isfinite(cutoff_hz), "cutoff must be positive and finite"
    assert 0.0 <= second_pole <= 1.0, "second_pole runs from 0 (one section) to 1 (two equal sections)"
    q2 = second_pole * second_pole
    x = 2.0 / ((1.0 + q2) + math.sqrt((1.0 + q2) ** 2 + 4.0 * q2))
    lower = cutoff_hz / math.sqrt(x)
    higher = lower / second_pole if second_pole > 0.0 else math.inf
    assert lower >= cutoff_hz and higher >= lower, "both corners lie at or above the cutoff"
    return lower, higher


def section_gains(cutoffs_hz: Sequence[float], second_pole: float, step_s: float) -> np.ndarray:
    """Per-section, per-channel update gains: row 0 is the lower corner, row 1 the higher."""
    assert len(cutoffs_hz) == CHANNELS, "one cutoff per channel"
    assert step_s > 0.0 and math.isfinite(step_s), "step must be positive"
    gains = np.empty((SECTIONS, CHANNELS))
    for ch, cutoff in enumerate(cutoffs_hz):
        for row, corner in enumerate(corners_hz(cutoff, second_pole)):
            gains[row, ch] = 1.0 - math.exp(-2.0 * math.pi * corner * step_s)
    assert np.all(gains > 0.0) and np.all(gains <= 1.0), "a gain left (0, 1]"
    return gains


class ImuFilter:
    """Two one-pole sections in series over the six IMU channels, stepped at the physics rate."""

    def __init__(self, accel_cutoff_hz: float, gyro_cutoff_hz: float, second_pole: float, step_s: float) -> None:
        self.gains = section_gains([accel_cutoff_hz] * 3 + [gyro_cutoff_hz] * 3, second_pole, step_s)
        self.state = np.zeros((SECTIONS, CHANNELS))
        self.updates = 0

    def reset(self, raw: np.ndarray) -> None:
        """Start both sections at a reading, as a robot held still long enough would be."""
        assert raw.shape == (CHANNELS,) and np.all(np.isfinite(raw)), "reset needs six finite values"
        self.state[:] = raw

    def update(self, raw: np.ndarray) -> None:
        """One physics step."""
        assert raw.shape == (CHANNELS,), "update needs six values"
        self.state[0] += self.gains[0] * (raw - self.state[0])
        self.state[1] += self.gains[1] * (self.state[0] - self.state[1])
        self.updates += 1

    @property
    def output(self) -> np.ndarray:
        return self.state[1]
