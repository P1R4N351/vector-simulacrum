"""Replay archived shelf trajectories with an accelerometer on the free cube."""
import csv
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
repo = here.parents[1]
sys.path[:0] = [str(here.parent / "p2y8-printed-fork-contact"), str(repo)]
from evaluate import run, observed
from simulacrum import cube, mjcf, params


def main():
    v = params.values(params.load(model="vector_2"))
    xml = ET.parse(sys.argv[2])
    xml.find(".//body[@name='cube']").append(ET.fromstring(mjcf._cube_chip_xml(v)))
    ET.SubElement(ET.SubElement(xml.getroot(), "sensor"), "accelerometer", name="accel", site="cube_chip")
    fig, axes = plt.subplots(5, 1, figsize=(8,8), sharex=True, sharey=True)
    with tempfile.NamedTemporaryFile(suffix=".xml") as fixture, (here / "movement.csv").open("w") as f:
        xml.write(fixture.name)
        sys.argv[2] = fixture.name
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["trial","physical_net_mm","sim_net_mm","peak_observed_packet_score","start_events","stop_events"])
        for trial, ax in enumerate(axes, 1):
            sensor = cube.CubeAccelerometer(v,.001,seed=trial)
            trace = []
            def step(model,data,sec,lift):
                sensor.step(data.time, data.sensor("accel").data / np.linalg.norm(model.opt.gravity))
                if len(trace) == 0 or data.time >= trace[-1][0]+.00999:
                    trace.append((data.time, sensor.movement.score))
            _, path = run((trial,"stl","overhead","weld",0,0,0),step,observer_stride=1)
            score = np.array(trace)
            # The final packet of every tick is processed, even between plotted samples.
            events = sensor.movement.events
            net = np.linalg.norm(path[-1][1:3])
            writer.writerow([trial, observed["over_magnitude"][trial-1],net,score[:,1].max(),
                             sum(start for _,start in events),sum(not start for _,start in events)])
            ax.plot(score[:,0]-1.5,score[:,1],color="#286373")
            ax.axhline(10000,color="#a63f29",ls="--")
            ax.set_ylabel(f"Trial {trial}")
    axes[-1].set_xlabel("Seconds from approach command")
    fig.suptitle("Simulated movement score and stock start threshold")
    fig.tight_layout()
    fig.savefig(here / "movement.png",dpi=150)


if __name__ == "__main__":
    main()
