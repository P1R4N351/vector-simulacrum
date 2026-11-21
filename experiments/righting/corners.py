"""Separate the eight rubber corner caps from the hard body of a 44 mm cube."""
from itertools import product
import numpy as np

def split_caps(boxes, size_mm=8.0, proud_mm=0.15):
    """Yield (center, half-size, material). Keep the original recesses between caps."""
    limit = .022 - size_mm * .001
    for center, half in boxes:
        low = np.subtract(center, half)
        high = np.add(center, half)
        axes = []
        for a,b in zip(low,high):
            cuts = [a, *[k for k in (-limit,limit) if a < k < b], b]
            axes.append(list(zip(cuts,cuts[1:])))
        parts = list(product(*axes))
        if not any(all(abs((a+b)/2) > limit for a,b in part) for part in parts):
            yield center,half,"shell"
            continue
        for part in parts:
            left,right = np.array(part).T.copy()
            cap = np.all(abs((left+right)/2) > limit)
            if cap:
                left[np.isclose(left,-.022)] -= proud_mm*.001
                right[np.isclose(right,.022)] += proud_mm*.001
            yield ((left+right)/2).tolist(), ((right-left)/2).tolist(), "rubber" if cap else "shell"
