from evaluate import run, here, ProcessPoolExecutor, csv

if __name__ == "__main__":
    jobs = [
        (i, "stl", "overhead", "weld", 0, 0, 0, mu)
        for mu in (1, 2, 4)
        for i in range(1, 6)
    ]
    with ProcessPoolExecutor(4) as pool, (here / "materials.csv").open("w") as f:
        out = csv.writer(f, lineterminator="\n")
        with (here / "scores.csv").open() as source:
            header = next(csv.reader(source))
        out.writerow(header[:7] + ["fork_friction_multiplier"] + header[7:])
        for row, _ in pool.map(run, jobs):
            out.writerow(row)
