"""Run the simulacrum once and print what happened..

    the GPU host python demo.py

Three short runs: a straight drive and a turn on the table, then a drive at
the table edge at each control level. Prints numbers, draws nothing.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from simulacrum import params  # noqa: E402
from simulacrum.robot import CONTROL_DEFAULT, CONTROL_OVERRIDE, VectorSim  # noqa: E402

EDGE_START_MM = 150.0
EDGE_DRIVE_MM = 300.0
EDGE_SPEED_MMPS = 100.0


def nominal_run() -> None:
    sim = VectorSim()
    assert sim.state.cliff_detected is False, "robot starts on the table"
    drive = sim.drive_straight(200.0, 100.0)
    x, y, _ = sim.pose_mm()
    print(f"drive 200 mm at 100 mm/s: {drive.result}, ended at x={x:.2f} y={y:.2f} mm, "
          f"{drive.ticks * sim.dt:.2f} s")
    turn = sim.turn_in_place(90.0)
    print(f"turn 90 deg: {turn.result}, turned {turn.turned_deg:.2f} deg, {turn.ticks * sim.dt:.2f} s")
    assert drive.result == "completed" and turn.result == "completed", "nominal motion did not complete"


def edge_run(level: str) -> None:
    v = params.values(params.load())
    start_x = 0.5 * v["table_length_mm"] - EDGE_START_MM
    sim = VectorSim(control_level=level, start_xy_mm=(start_x, 0.0))
    result = sim.drive_straight(EDGE_DRIVE_MM, EDGE_SPEED_MMPS)
    sim.advance(2.0)
    truth = sim.sim_truth()
    print(f"edge drive, control level {level}: {result.result}, moved {result.moved_mm:.1f} mm, "
          f"fallen={truth['fallen']}, margin to edge {truth['edge_margin_mm']:.1f} mm, "
          f"reactions={sim.reactions}, has_control={sim.has_control}")
    assert truth["fallen"] == (level == CONTROL_OVERRIDE), "edge outcome differs from the control level's"
    assert (sim.reactions == 1) == (level == CONTROL_DEFAULT), "reaction count differs from the control level's"


def main() -> int:
    counts = params.summary(params.load())
    print("parameters:", counts)
    nominal_run()
    edge_run(CONTROL_DEFAULT)
    edge_run(CONTROL_OVERRIDE)
    return 0


if __name__ == "__main__":
    sys.exit(main())
