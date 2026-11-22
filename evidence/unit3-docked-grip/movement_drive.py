"""Compare movement score and isolate magnitude and direction changes in packet space."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from face_contact import start_raise
from simulacrum.cube import MovementListener, engine_samples, packet

root = Path(__file__).resolve().parent


def compare(packets):
    listeners = [MovementListener() for _ in range(3)]
    baseline = np.median([sample for t, payload in packets if t < 0
                          for sample in engine_samples(payload)], axis=0)
    length = np.linalg.norm(baseline)
    direction = baseline / length
    rows = []
    for t, payload in packets:
        for sample in engine_samples(payload):
            norm = np.linalg.norm(sample)
            # Synthetic controls hold one property of the sampled specific force fixed.
            controls = (sample, direction * norm, sample * (length / norm))
            for listener, control in zip(listeners, controls):
                listener.update(t, control)
            rows.append([t, *(listener.score for listener in listeners)])
    return np.asarray(rows)


def main():
    data = np.loadtxt(root/'cube.csv', delimiter=',', skiprows=1).reshape(-1, 3, 5)
    series = {'P2Y8': [(p[0, 0], packet(0, p[:, 2:].astype(int).tolist())) for p in data]}
    for gap in (0, 1, 2):
        sim, _ = start_raise(gap)
        start = sim.data.time
        sim.advance(2.45)
        series[f'{gap} mm gap'] = [(t-start, payload) for t, payload in sim.cube.messages]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True, layout='constrained')
    rows = []
    for label, packets in series.items():
        trace = compare([(t, p) for t, p in packets if -.21 <= t <= 2.45])
        trace = trace[trace[:, 0] >= 0]
        axes[0].plot(trace[:, 0], trace[:, 1], '.' if label == 'P2Y8' else '-', ms=3, label=label)
        for i, component in enumerate(('Full signal', 'Magnitude only', 'Direction only'), 1):
            if label == 'P2Y8':
                axes[1].plot(trace[:, 0], trace[:, i], '.', ms=3, label=component)
            peak = np.argmax(trace[:, i])
            rows.append([label, component, trace[peak, i], trace[peak, 0],
                         np.count_nonzero(trace[:, i] > 10000)])
    for ax, title in zip(axes, ('Free-cube replay', 'Physical packet controls')):
        ax.axhline(10000, color='black', ls=':', lw=1, label='Motion start threshold')
        ax.set(title=title, xlabel='Seconds from lift command')
        ax.legend(fontsize=8)
    axes[0].set_ylabel('Movement listener score')
    fig.savefig(root/'movement-drive.png', dpi=165)
    with (root/'movement-drive.csv').open('w') as f:
        w = csv.writer(f)
        w.writerow(['source','signal','peak_score','peak_packet_s','slots_above_start_threshold'])
        w.writerows(rows)


if __name__ == '__main__':
    main()
