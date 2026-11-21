"""Compare equal 2 mm registration changes on opposite sides of the contact."""
import csv
from reach import here, score


if __name__ == "__main__":
    with (here / "depth-check.csv").open("w") as f:
        out = csv.writer(f, lineterminator="\n")
        out.writerow(["trial", "reach_mm", "cube_depth_mm", "prelift_error_mm",
                      "endpoint_error_mm", "peak_tilt_deg"])
        for trial in range(1, 6):
            for reach, depth in ((0, 0), (2, 0), (0, -2)):
                _, _, pre, end, tilt = score((trial, reach), cube_depth_mm=depth)
                out.writerow([trial, reach, depth, pre, end, tilt])
