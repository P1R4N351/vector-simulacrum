"""Is the spin error aliasing? The chip's true force at every physics step against the cube's samples.
Authored-by: CPCS (cpcs-66ffc23). ./run_in_ct.sh evidence/cube-model-20261121/alias.py"""
from __future__ import annotations
import json, math, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from simulacrum import cube, params  # noqa: E402
from simulacrum.robot import VectorSim  # noqa: E402

DOC = params.load()


def run(mass: float, tick_ms: float) -> dict:
    sim = VectorSim(values=params.values(DOC, {"cube_mass_g": mass, "cube_tick_ms": tick_ms}), cube="lift", noise_seed=6)
    forces: list = []
    inner = sim.cube.step
    sim.cube.step = lambda t, f: (forces.append(np.array(f, dtype=float)), inner(t, f))[1]
    sim.advance(1.0)
    n_still_f, n_still_m = len(forces), len(sim.cube.messages)
    yaw0, t0 = sim.heading_total_rad(), float(sim.data.time)
    sim.set_wheel_motors(-80.0, 80.0)
    sim.advance(2.0)
    first_f, first_m = len(forces), len(sim.cube.messages)
    y1, t1 = sim.heading_total_rad(), float(sim.data.time)
    sim.advance(2.0)
    omega = (sim.heading_total_rad() - y1) / (float(sim.data.time) - t1)
    true_still = np.mean(forces[:n_still_f], axis=0)
    spin_f = np.array(forces[first_f:])
    true_delta = spin_f.mean(axis=0) - true_still
    cpg = sim.v["cube_accel_counts_per_g"]
    s_still = cube.samples_g(cube.rows(sim.cube.messages[:n_still_m]), cpg).mean(axis=0)
    s_spin = cube.samples_g(cube.rows(sim.cube.messages[first_m:]), cpg).mean(axis=0)
    spec = np.abs(np.fft.rfft(spin_f - spin_f.mean(axis=0), axis=0)) / len(spin_f)
    freqs = np.fft.rfftfreq(len(spin_f), d=0.001)
    top = {ax: [[round(float(freqs[i]), 1), round(float(2 * spec[i, k]), 4)] for i in np.argsort(spec[:, k])[::-1][:4]]
           for k, ax in enumerate("xyz")}
    g = sim.v["gravity_mmps2"]
    return {"cube_mass_g": mass, "cube_tick_ms": tick_ms, "omega_radps": round(omega, 4),
            "true_delta_g": true_delta.round(5).tolist(), "sampled_delta_g": (s_spin - s_still).round(5).tolist(),
            "true_centre_mm": (true_delta[:2] * g / omega ** 2).round(2).tolist(),
            "sampled_centre_mm": ((s_spin - s_still)[:2] * g / omega ** 2).round(2).tolist(),
            "true_force_rms_g_turning": spin_f.std(axis=0).round(4).tolist(), "largest_lines_hz_g": top}


for args in ((25.0, 10.0), (25.0, 10.07)):
    print(json.dumps(run(*args)))
