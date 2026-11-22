"""Run the recorded schedule through the unmodified Vector 2.0 replay."""
import csv
import gzip
import json
import math
from pathlib import Path
from statistics import median

from simulacrum import params
from simulacrum.robot import VectorSim

folder = Path(__file__).resolve().parent
with gzip.open(folder / "paired.jsonl.gz", "rt") as stream:
    records = [json.loads(line) for line in stream]
physical = [row for row in records if row["kind"] == "state"]
commands = [row for row in records if row["kind"] == "command"]
arrivals = {row["n"]: row["t"] for row in records if row["kind"] == "arrived"}
sim = VectorSim(params.values(params.load(model="vector_2")), charger_contact=True,
                lift_mm=physical[0]["lift"], head_deg=math.degrees(physical[0]["head"]))
t0 = commands[0]["t"]
tick = 0
next_cmd = 0
samples = []
while t0 + tick * sim.dt <= physical[-1]["t"]:
    if next_cmd < len(commands) and t0 + tick * sim.dt >= commands[next_cmd]["t"]:
        assert sim.command_lift_height(32+60*commands[next_cmd]["pos"], duration_s=.7)
        next_cmd += 1
    samples.append(sim.step().lift_height_mm)
    tick += 1
with (folder / "holds.csv").open() as stream:
    holds = list(csv.DictReader(stream))
with (folder / "replay.csv").open("w") as stream:
    writer = csv.writer(stream)
    writer.writerow(["raise", "start_fraction", "physical_mm", "sim_mm", "sim_minus_physical_mm"])
    for command, hold in zip((c for c in commands if c["pos"] == 1), holds):
        end = arrivals[command["n"]]
        times = [r["t"] for r in physical if .12 <= r["t"]-end <= .6]
        sim_mm = median(samples[round((t-t0)/sim.dt)-1] for t in times)
        mm = float(hold["median_mm"])
        writer.writerow([hold["raise"], hold["start_fraction"], mm, sim_mm, sim_mm-mm])
