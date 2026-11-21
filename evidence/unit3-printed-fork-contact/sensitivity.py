from evaluate import run, here, ProcessPoolExecutor, csv

if __name__ == "__main__":
    jobs = [
        (1, "stl", h, "weld", x, y, z)
        for h in ("rgb", "overhead")
        for x in (-6, -3, 0, 3)
        for y in (-4, 0, 4)
        for z in (-1.5, 0)
    ]
    with ProcessPoolExecutor(4) as pool, (here / "sensitivity.csv").open("w") as f:
        out = csv.writer(f, lineterminator="\n")
        with (here / "scores.csv").open() as source:
            out.writerow(next(csv.reader(source)))
        for row, _ in pool.map(run, jobs):
            out.writerow(row)
