"""Replay each three-move sequence starting from its measured preparatory pose."""
import csv
import gzip
import json
from math import degrees
from pathlib import Path
from statistics import median

from simulacrum import params
from simulacrum.robot import VectorSim

folder = Path(__file__).resolve().parent
with gzip.open(folder / "physical.jsonl.gz", "rt") as stream:
    events = [json.loads(line) for line in stream]
records = [r for r in events if r["kind"] == "state"]
values = params.values(params.load(model="vector_2"))
with (folder / "replay.csv").open("w") as stream:
    writer = csv.writer(stream)
    writer.writerow(["block", "prep_fraction", "phase", "measured_hold_mm", "sim_hold_mm", "sim_minus_real_mm"])
    for start in [r for r in events if r["kind"] == "command" and r["phase"] == "prepare"]:
        pose = next(r for r in reversed(records) if r["t"] < start["t"])
        sim = VectorSim(values, head_deg=degrees(pose["head"]), lift_mm=pose["lift"], charger_contact=True)
        sequence = [r for r in events if r.get("block") == start["block"] and r.get("prep") == start["prep"]]
        ticks = 0
        def advance(t):
            global ticks
            target = round((t - start["t"]) / sim.dt)
            if target > ticks:
                sim.step(target - ticks)
                ticks = target
        for e in sequence:
            advance(e["t"])
            if e["kind"] == "command":
                assert sim.command_lift_height(32 + 60 * e["pos"], duration_s=.7)
                continue
            window = [r for r in records if .12 <= r["t"] - e["t"] <= .6]
            predicted = []
            for sample in window:
                advance(sample["t"])
                predicted.append(sim.states[-1].lift_height_mm)
            measured = median(r["lift"] for r in window)
            writer.writerow([e["block"],e["prep"],e["phase"],measured,median(predicted),median(predicted)-measured])
