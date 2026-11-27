"""Select front support softness on W4/W8/W25 and transfer it to four other mounts."""

import csv
from pathlib import Path
from tempfile import TemporaryDirectory
import matplotlib.pyplot as plt
import numpy as np
from front_contact import replay_front_contact
from earlier_inputs import HERE

TRAIN = ("W4", "W8", "W25")
TEST = ("W26", "W27", "W29", "W30")
CANDIDATES = (4, 8, 16, 32, 64)


def evaluate(ms, trials):
    with TemporaryDirectory() as tmp:
        work = Path(tmp) / "scores.csv"
        replay_front_contact(ms, trials, work)
        with work.open() as f:
            return list(csv.DictReader(f))


def errors(rows):
    counts = [abs(int(r["predicted_count"]) - int(r["measured_count"])) for r in rows]
    times = [
        abs(float(r["first_error_ms"])) if r["first_error_ms"] else 900 for r in rows
    ]
    return np.mean(counts), np.mean(times)


def main():
    train = {ms: evaluate(ms, TRAIN) for ms in CANDIDATES}
    best = min(CANDIDATES, key=lambda ms: (*errors(train[ms]), ms))
    test = {ms: evaluate(ms, TEST) for ms in sorted({4, best})}
    fig, axes = plt.subplots(1, 2, figsize=(9, 4), layout="constrained")
    for i, label in enumerate(
        ("Mean absolute tap count error", "Mean absolute first tap error (ms)")
    ):
        ax = axes[i]
        ax.plot(
            CANDIDATES,
            [errors(train[ms])[i] for ms in CANDIDATES],
            ".-",
            label="Selection runs",
        )
        ax.plot(
            list(test),
            [errors(r)[i] for r in test.values()],
            "s--",
            label="Transfer runs",
        )
        ax.axvline(best, color="grey", ls=":")
        ax.set(xlabel="Front support time constant (ms)", ylabel=label)
        ax.legend()
    fig.savefig(HERE / "tread-softness.png", dpi=160)
    with (HERE / "tread-softness.csv").open("w") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["split", "support_ms", *train[4][0].keys()])
        for split, result in [("selection", train), ("transfer", test)]:
            for ms, rows in result.items():
                print(split, ms, errors(rows))
                for row in rows:
                    w.writerow([split, ms, *row.values()])


if __name__ == "__main__":
    main()
