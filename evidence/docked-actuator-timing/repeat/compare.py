"""Apply the original onset and endpoint checks to the three docked recordings."""
from pathlib import Path
import sys
here = Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent))
import analyze
analyze.FILES = {"unit 1":here/"unit1.jsonl","unit 2":here.parent/"unit2.jsonl","unit 3":here/"unit3.jsonl"}
analyze.HERE = here
analyze.main()
