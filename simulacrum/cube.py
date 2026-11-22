"""The light cube's accelerometer, as the cube's chip and app make its messages..

Needs numpy, not MuJoCo. robot.py feeds it the specific force at the cube's chip site once per physics
step when the model carries a cube; on its own it takes any specific-force series.

The chain, in the order the hardware runs it. Physical numbers come from parameters.json; the constants
below are data formats and firmware logic, cited where they are defined.

1. Specific force at the chip, in g, on the chip's axes. Bias and scale (cube_accel_bias_mg,
   cube_accel_scale), and white noise injected before the filter: per physics step h, a standard deviation
   of n * sqrt(1 / (2h)) for the one-sided density n (cube_accel_noise_density_ug), which is white up to
   the step's Nyquist frequency and leaves the filter's passband at density n.
2. The chip's bandwidth filter, "2nd order" (BMA253DS:404-410), as two one-pole sections whose series
   response is -3 dB at cube_accel_bandwidth_hz (simulacrum/imu_filter.py; cube_accel_filter_second_pole
   is the ratio of their corners). Run at the physics step: with h = 1 ms its corners sit near the step's
   Nyquist frequency, so its shape above about 100 Hz is approximate.
3. A frame every cube_accel_frame_period_ms (filtered data at twice the bandwidth, BMA253DS:845-847,
   Table 4 at :866: 2 ms at 250 Hz). Quantised to 12 bits, two's complement (BMA253DS:832), clipped to the
   range, stored left-justified in 16 bits (MSB holds bits 11..4, the LSB byte bits 3..0 above four bits
   that "may have any value and should be ignored", BMA253DS:832-842); the nominal model writes those four as 0. Unit 3 uses its observed 4k+1 packet codes.
4. FIFO mode (accel.c writes FIFO_CONFIG_1 = 0x40: fifo_mode 01, all three axes, BMA253DS:2192-2193,
   2216-2217): up to 32 frames, and once full "the data collection is stopped, and all additional samples
   are ignored" (BMA253DS:2174-2178).
5. The app's tick, every cube_tick_ms: it reads every frame in the FIFO into the same slot, so the slot keeps
   the last frame read; a tick that finds no frame leaves the slot as it was. Each frame's z goes through
   the tap counter. With app="mean" (cube_firmware/tick_avg.s, a lane's replacement tick) the slot
   gets the mean of the tick's frames instead, with its fraction in the four low bits. After the third tick it sends the 20-byte message (protocol.h AccelDataCommand:
   command 2, tap_count, three samples of x, y, z as int16 little-endian) and starts the next. accel.c
   hal_acc_tick; the shipped app's code at 0x20005db4..0x20005e72 does the same, constant for constant
   (evidence/cube-firmware-20261121/PINMAP.md).

Not modelled: the battery messages (adc.c), the cube's lights, the radio. The chip's clock and the app's
tick run free of each other; their phases are arguments.
"""

from __future__ import annotations

import math
import random
import struct
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import imu_filter

COMMAND_ACCEL = 2           # protocol.h
FRAMES_PER_MESSAGE = 3      # protocol.h ACCEL_FRAMES_PER_MSG
FIFO_FRAMES = 32            # BMA253DS:2169 "capable of storing up to 32 frames"
DATA_BITS = 12              # BMA253DS:832 "The width of acceleration data is 12 bits"
LEFT_SHIFT = 16 - DATA_BITS
TAP_THRESH = 90 * 32        # accel.c TAP_THRESH; the shipped app builds it as 45 << 6
TAP_DEBOUNCE = 90           # accel.c TAP_DEBOUNCE ("90ms": it counts frames, which are 2 ms apart)
TAP_DURATION = 10           # accel.c TAP_DURATION
MAX_EVENTS_PER_STEP = 64
TIME_EPS_S = 1e-9           # events this close count as simultaneous; far below any period here
AXES = 3


def int16(value: int) -> int:
    """Wrap to a signed 16-bit value, as the C code's int16_t does."""
    return ((int(value) + 0x8000) & 0xFFFF) - 0x8000


class TapCounter:
    """accel.c's tap detector, ported as shipped. Only the count leaves the cube; tapPos, tapNeg and tapTime
    are computed by the C code and never used, so they are left out."""

    def __init__(self) -> None:
        self.last, self.debounce, self.pos_first, self.count = 0, 0, False, 0

    def feed(self, z: int) -> None:
        """One FIFO frame's z, in raw counts."""
        diff = int16(z - self.last)
        self.last = int16(z)
        if self.debounce > 0:
            # As shipped: after a negative-first edge the comparison is with TAP_DEBOUNCE (90 counts), not a
            # threshold, so a sharp drop counts as a tap at the next frame that does not rise by 90.
            limit = -TAP_THRESH if self.pos_first else TAP_DEBOUNCE
            if self.debounce > TAP_DEBOUNCE - TAP_DURATION and diff < limit:
                self.count = (self.count + 1) & 0xFF
                self.debounce = TAP_DEBOUNCE - TAP_DURATION
            self.debounce -= 1
            return
        if diff > TAP_THRESH:
            self.pos_first = True
        elif diff < -TAP_THRESH:
            self.pos_first = False
        else:
            return
        self.debounce = TAP_DEBOUNCE


def counts_of(force_g: np.ndarray, lsb_per_g: float, shift: int = LEFT_SHIFT, low_bits: int = 0) -> List[int]:
    """Quantize and saturate on the signed word lattice of the selected cube."""
    full = 1 << (15 - shift)
    out = []
    for value in force_g:
        q = int(round(float(value) * lsb_per_g))
        out.append((max(-full, min(full - 1, q)) << shift) | low_bits)
    return out


def packet(tap_count: int, slots: Sequence[Sequence[int]]) -> bytes:
    """The app's accelerometer message, laid out as protocol.h's AccelDataCommand."""
    assert len(slots) == FRAMES_PER_MESSAGE and all(len(s) == AXES for s in slots), "three samples of x, y, z"
    flat = [int16(c) for s in slots for c in s]
    return struct.pack("<BB9h", COMMAND_ACCEL, tap_count & 0xFF, *flat)


def mean_counts(values: Sequence[int]) -> int:
    """The mean of a tick's frames on one axis, as cube_firmware/tick_avg.s computes it: the 12-bit values (four
    low bits dropped), summed, times 16 over the count, halves rounded up. The result is left-justified like
    the chip's own counts, with the mean's fraction in the four low bits."""
    assert len(values) > 0, "a mean needs at least one frame"
    total = sum(int(v) >> LEFT_SHIFT for v in values)
    return (16 * total + len(values) // 2) // len(values)


APPS = ("last", "mean")  # the shipped app, and cube_firmware/tick_avg.c


class CubeApp:
    """The app's tick. Every FIFO frame's z goes through the tap counter; the tick keeps one value per axis
    (app "last": the last frame, as shipped; app "mean": the mean of the tick's frames); a tick with no frame
    leaves its slot as it was; after the third tick the message goes out."""

    def __init__(self, app: str = "last") -> None:
        assert app in APPS, "the app is last or mean"
        self.app = app
        self.slots = [[0, 0, 0] for _ in range(FRAMES_PER_MESSAGE)]
        self.slot = 0
        self.taps = TapCounter()
        self.messages: List[Tuple[float, bytes]] = []

    def tick(self, t_s: float, frames: Sequence[Sequence[int]]) -> None:
        for frame in frames:
            self.taps.feed(frame[2])
        if frames and self.app == "last":
            self.slots[self.slot] = list(frames[-1])
        elif frames:
            self.slots[self.slot] = [mean_counts([f[axis] for f in frames]) for axis in range(AXES)]
        self.slot += 1
        if self.slot >= FRAMES_PER_MESSAGE:
            self.messages.append((t_s, packet(self.taps.count, self.slots)))
            self.slot = 0


def engine_samples(payload: bytes) -> np.ndarray:
    """Decode packet slots as mm/s² with CubeAccelComponent's float conversion."""
    assert len(payload) == 20 and payload[0] == COMMAND_ACCEL
    slots = np.frombuffer(payload, dtype="<i2", offset=2).reshape(FRAMES_PER_MESSAGE, AXES)
    return slots.astype(np.float32) / 32767 * 4 * 9810


class TapListener:
    """Packet-level ObjectTapped candidates before engine tap grouping or SDK delivery."""

    def __init__(self) -> None:
        self.previous: Optional[int] = None
        self.events: List[float] = []

    def receive(self, t_s: float, payload: bytes) -> None:
        assert len(payload) == 20 and payload[0] == COMMAND_ACCEL
        count = payload[1]
        if self.previous is not None and count != self.previous:
            self.events.append(t_s)
        self.previous = count


class MovementListener:
    """Hardware movement listener before the engine's carrying/docking broadcast checks."""

    def __init__(self) -> None:
        self.previous: Optional[np.ndarray] = None
        self.filtered = np.zeros(AXES, dtype=np.float32)
        self.score = 0.0
        self.moving = False
        self.events: List[Tuple[float, bool]] = []

    def update(self, t_s: float, accel: np.ndarray) -> None:
        if self.previous is not None:
            self.filtered = np.float32(.8) * (self.filtered + accel - self.previous)
            magnitude = float(np.linalg.norm(self.filtered))
            self.score = float(np.clip(np.float32(self.score + min(500., magnitude)) - 300., 0., 20000.))
            if self.score > 10000. and not self.moving:
                self.moving = True
                self.events.append((t_s, True))
            elif 0. < self.score < 5000. and self.moving:
                self.moving = False
                self.events.append((t_s, False))
        self.previous = accel


class UpAxisListener:
    """Engine-side UpAxisChangedListener. Axis codes follow gateway/public/cube.proto."""

    def __init__(self) -> None:
        self.filtered: Optional[np.ndarray] = None
        self.axis = 0
        self.events: List[Tuple[float, int]] = []

    def receive(self, t_s: float, payload: bytes) -> None:
        for accel in engine_samples(payload):
            self.update(t_s, accel)

    def update(self, t_s: float, accel: np.ndarray) -> None:
        if self.filtered is None:
            self.filtered = accel
            return
        self.filtered = accel * np.float32(.03) + self.filtered * np.float32(.97)
        index = int(np.argmax(np.abs(self.filtered)))
        dominant = self.filtered[index]
        if dominant == 0:
            return
        axis = 1 + 2 * index + int(dominant > 0)
        if self.axis and axis != self.axis:
            self.events.append((t_s, axis))
        self.axis = axis


class CubeAccelerometer:
    """The cube's chip and app, stepped at the physics rate. Messages are (send time s, 20 bytes)."""

    def __init__(self, v: Dict[str, Any], step_s: float, seed: int = 0, frame_phase_s: float = 0.0,
                 tick_phase_s: float = 0.0, app: str = "last") -> None:
        assert step_s > 0.0 and math.isfinite(step_s), "step must be positive"
        assert app in APPS, "the app is last or mean"
        self.app_kind = app
        shift, low_bits = v.get("cube_accel_word_format", [LEFT_SHIFT, 0])
        self.word_shift, self.low_bits = int(shift), int(low_bits)
        if (shift != self.word_shift or low_bits != self.low_bits or
                not 0 <= shift < 16 or not 0 <= low_bits < (1 << self.word_shift)):
            raise ValueError("Invalid cube accelerometer word format")
        self.lsb_per_g = v["cube_accel_counts_per_g"] / (1 << self.word_shift)
        assert abs(v["cube_accel_counts_per_g"] * v["cube_accel_range_g"] - 32768) < 1e-9, \
            "counts per g and range must span the signed word"
        self.frame_s = v["cube_accel_frame_period_ms"] * 0.001
        self.tick_s = v["cube_tick_ms"] * 0.001
        assert self.frame_s > 0.0 and self.tick_s > 0.0, "periods must be positive"
        assert 0.0 <= frame_phase_s < self.frame_s and 0.0 <= tick_phase_s < self.tick_s, "phase out of range"
        cutoff = v["cube_accel_bandwidth_hz"]
        self.filter = imu_filter.ImuFilter(cutoff, cutoff, v["cube_accel_filter_second_pole"], step_s)
        self.scale = np.array(v["cube_accel_scale"], dtype=float)
        self.bias_g = np.array(v["cube_accel_bias_mg"], dtype=float) * 0.001
        self.noise_g = v["cube_accel_noise_density_ug"] * 1e-6 * math.sqrt(0.5 / step_s)
        assert self.scale.shape == (AXES,) and self.bias_g.shape == (AXES,), "scale and bias have three axes"
        self.noise_axis_scale = np.asarray(v.get("cube_accel_noise_axis_scale", [1., 1., 1.]))
        assert self.noise_axis_scale.shape == (AXES,) and np.all(np.isfinite(self.noise_axis_scale))
        assert np.all(self.noise_axis_scale >= 0.)
        self._rng = random.Random(seed)
        self._phases = (frame_phase_s, tick_phase_s)
        self.reset(np.array([0.0, 0.0, 1.0]), 0.0)

    def reset(self, force_g: np.ndarray, t_s: float) -> None:
        """Start at rest at a reading, with an empty FIFO and the clocks counted from t_s."""
        held = self._sensed(np.asarray(force_g, dtype=float), noise=False)
        self.filter.reset(np.concatenate([held, held]))
        self.fifo: List[List[int]] = []
        self.dropped = 0
        self.app = CubeApp(self.app_kind)
        self.taps = TapListener()
        self.up_axis = UpAxisListener()
        self.movement = MovementListener()
        # Event times are counted, not summed: ten 2 ms steps summed come to 0.020000000000000004 s and would
        # put the frame due at 20 ms after the tick due then.
        self._t0, self._frames, self._ticks = t_s, 0, 0

    def _frame_time(self) -> float:
        return self._t0 + self._phases[0] + (self._frames + 1) * self.frame_s

    def _tick_time(self) -> float:
        return self._t0 + self._phases[1] + (self._ticks + 1) * self.tick_s

    def _sensed(self, force_g: np.ndarray, noise: bool) -> np.ndarray:
        assert force_g.shape == (AXES,) and np.all(np.isfinite(force_g)), "specific force has three finite axes"
        out = force_g * self.scale + self.bias_g
        if noise and self.noise_g > 0.0:
            out = out + self.noise_axis_scale * [self._rng.gauss(0.0, self.noise_g) for _ in range(AXES)]
        return out

    def step(self, t_s: float, force_g: np.ndarray) -> None:
        """One physics step ending at t_s. Frames and ticks due by then happen in time order, a frame first
        when both fall at the same moment, and each frame holds the filter's output at this step."""
        sensed = self._sensed(np.asarray(force_g, dtype=float), noise=True)
        self.filter.update(np.concatenate([sensed, sensed]))
        frame = counts_of(self.filter.output[:AXES], self.lsb_per_g, self.word_shift, self.low_bits)
        for _ in range(MAX_EVENTS_PER_STEP):
            frame_at, tick_at = self._frame_time(), self._tick_time()
            if min(frame_at, tick_at) > t_s + TIME_EPS_S:
                return
            if frame_at <= tick_at + TIME_EPS_S:
                self._store(frame)
                self._frames += 1
            else:
                self._tick(tick_at)
                self._ticks += 1
        raise AssertionError("more chip events in one physics step than the step can hold")

    def _store(self, frame: List[int]) -> None:
        if len(self.fifo) >= FIFO_FRAMES:
            self.dropped += 1
            return
        self.fifo.append(list(frame))

    def _tick(self, t_s: float) -> None:
        self.app.tick(t_s, self.fifo)
        if self.app.slot == 0:
            payload = self.app.messages[-1][1]
            self.taps.receive(t_s, payload)
            for accel in engine_samples(payload):
                self.up_axis.update(t_s, accel)
                self.movement.update(t_s, accel)
        self.fifo = []

    @property
    def messages(self) -> List[Tuple[float, bytes]]:
        return self.app.messages


def rows(messages: Sequence[Tuple[float, bytes]]) -> List[Dict[str, Any]]:
    """Messages as the collector's "a" rows (hardware/cube_accel.py), so model and recording read alike."""
    out = []
    for t_s, data in messages:
        assert len(data) == 20 and data[0] == COMMAND_ACCEL, "not an accelerometer message"
        out.append({"k": "a", "t": round(float(t_s), 4), "tap": data[1],
                    "raw": list(struct.unpack_from("<9h", data, 2))})
    return out


def samples_g(accel_rows: Sequence[Dict[str, Any]], counts_per_g: float, app: str = "last", *,
              padding_bits: int = LEFT_SHIFT) -> np.ndarray:
    """Every sample in the rows, in g, one row per sample. Stock frames have four padding bits (BMA253DS:841).
    Use padding_bits=2 for the recorded unit 3 cube. The mean app uses those bits for the fractional part of the tick average. Each row's three samples are 10 ms apart, the last at the row's time."""
    assert counts_per_g > 0.0, "counts per g must be positive"
    assert app in APPS, "unknown cube app"
    assert isinstance(padding_bits, int) and 0 <= padding_bits < 16
    shift = padding_bits if app == "last" else 0
    out = []
    for row in accel_rows:
        raw = row["raw"]
        assert len(raw) == AXES * FRAMES_PER_MESSAGE, "a row carries nine counts"
        for i in range(FRAMES_PER_MESSAGE):
            out.append([((int(c) >> shift) << shift) / counts_per_g for c in raw[AXES * i:AXES * i + AXES]])
    return np.array(out, dtype=float).reshape(-1, AXES)


def turn_centre(still: np.ndarray, spinning: np.ndarray, omega_radps: float,
                gravity_mmps2: float) -> Dict[str, Any]:
    """Where a steady turn's centre lies from the chip, in the chip's horizontal axes, from samples (g).

    Turning steadily at omega, the chip feels omega^2 r towards the centre; gravity is the same in its frame
    as when still, and there is no tangential part. So the mean difference, divided by omega^2, is the vector
    from the chip to the centre. Assumes the chip's z is vertical and the body does not tilt in the turn;
    a tilt moves gravity into x and y and shows here as a shift of the centre.
    """
    assert still.ndim == 2 and spinning.ndim == 2 and still.shape[1] == AXES == spinning.shape[1], "(n, 3) samples"
    assert len(still) > 0 and len(spinning) > 0, "need samples both still and spinning"
    assert abs(omega_radps) > 0.1, "the turn rate is too small to divide by"
    delta_g = spinning.mean(axis=0) - still.mean(axis=0)
    toward = delta_g[:2] * gravity_mmps2 / (omega_radps * omega_radps)
    return {"delta_g": [float(x) for x in delta_g], "centre_mm": [float(x) for x in toward],
            "radius_mm": float(math.hypot(toward[0], toward[1])), "omega_radps": float(omega_radps),
            "samples": (len(still), len(spinning))}
