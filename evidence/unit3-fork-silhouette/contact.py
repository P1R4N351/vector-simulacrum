"""Usage: python contact.py REPOSITORY REGISTERED_XML"""
import csv
import sys
from pathlib import Path
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parent / "unit3-printed-fork-contact"))
import evaluate

original = evaluate.states.copy()
with (here / 'contact.csv').open('w') as output:
    writer = csv.writer(output)
    writer.writerow(['scale','trial','vector_error_mm','peak_tilt_deg'])
    for scale in [1, 0.8533177021533036]:
        evaluate.states['lift_mm'] = np.where(original['lift_mm'] > 32,
            32 + scale * (original['lift_mm'] - 32), original['lift_mm'])
        for trial in range(1,6):
            row, _ = evaluate.run((trial,'stl','overhead','weld',0,0,0))
            writer.writerow([scale,trial,row[-3],row[-1]])
