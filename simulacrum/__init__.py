"""Vector simulacrum: a MuJoCo model of the Anki/DDL Vector..

params and mjcf import without MuJoCo. robot needs it, so it is not imported
here: `from simulacrum import robot` where MuJoCo is installed.
"""

from . import mjcf, params

__all__ = ["mjcf", "params"]
