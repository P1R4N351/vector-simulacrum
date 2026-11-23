"""Tests for image_scan.py. The matching tests need no OCR; the end-to-end tests need tesseract and
Pillow and are skipped without them. Run: python3 -m unittest discover -s .github/leak-check -v
"""

import io
import json
import os
import re
import shutil
import sys
import tempfile
import unittest
import unittest.mock
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import image_scan  # noqa: E402

TOKEN = [("tok", re.compile(r"\bZQ4X\b"))]
HAVE_OCR = bool(shutil.which("tesseract"))
try:
    from PIL import Image
except ImportError:
    HAVE_OCR = False


def run_main(argv):
    out = io.StringIO()
    with redirect_stdout(out):
        code = image_scan.main(argv)
    return code, out.getvalue()


class Matching(unittest.TestCase):
    def test_plain_line(self):
        self.assertEqual(image_scan.match_ids(["legend ZQ4X measured"], TOKEN), ["tok"])

    def test_no_match(self):
        self.assertEqual(image_scan.match_ids(["legend unit 3 measured"], TOKEN), [])

    def test_dropped_gap_before_token(self):
        # tesseract renders a legend's "series NAME" as "seriesNAME"
        self.assertEqual(image_scan.match_ids(["seriesZQ4X"], TOKEN), ["tok"])

    def test_misread_y_as_yen(self):
        rx = [("y", re.compile(r"\bP7Y3\b"))]
        self.assertEqual(image_scan.match_ids(["P7\u00a53 | caption"], rx), ["y"])

    def test_each_id_reported_once(self):
        self.assertEqual(image_scan.match_ids(["ZQ4X", "ZQ4X again"], TOKEN), ["tok"])


class Selection(unittest.TestCase):
    def test_non_images_and_missing_files_dropped(self):
        with tempfile.TemporaryDirectory() as root:
            open(os.path.join(root, "a.PNG"), "wb").close()
            open(os.path.join(root, "b.txt"), "wb").close()
            got = image_scan.select_images(root, ["a.PNG", "b.txt", "gone.png"])
        self.assertEqual(got, ["a.PNG"])

    def test_nul_list(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, "list")
            with open(path, "wb") as fh:
                fh.write(b"x y.png\0z.jpg\0")
            self.assertEqual(image_scan.read_list(path), ["x y.png", "z.jpg"])


class Faults(unittest.TestCase):
    def test_missing_tesseract_is_a_fault_not_clean(self):
        saved = os.environ.get("PATH", "")
        os.environ["PATH"] = ""
        try:
            code, out = run_main(["--patterns", "unused", "x.png"])
        finally:
            os.environ["PATH"] = saved
        self.assertEqual(code, image_scan.EXIT_FAULT)
        self.assertIn("not a clean result", out)

    def test_usage(self):
        with redirect_stdout(io.StringIO()), unittest.mock.patch("sys.stderr", io.StringIO()):
            self.assertEqual(image_scan.main([]), image_scan.EXIT_USAGE)


@unittest.skipUnless(HAVE_OCR, "tesseract or Pillow not installed")
class EndToEnd(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp()
        self.patterns = os.path.join(self.root, "patterns.json")
        with open(self.patterns, "w") as fh:
            json.dump({"patterns": [{"id": "tok", "regex": r"\bHX7Z\b"}]}, fh)

    def tearDown(self):
        shutil.rmtree(self.root)

    def test_self_test_passes(self):
        self.assertEqual(run_main(["--patterns", self.patterns, "--self-test"])[0], 0)

    def test_finding_reports_id_not_text(self):
        from PIL import ImageDraw, ImageFont
        image_scan.render("series HX7Z canary", Image, ImageDraw, ImageFont).save(
            os.path.join(self.root, "hit.png"))
        code, out = run_main(["--patterns", self.patterns, "--root", self.root, "hit.png"])
        self.assertEqual(code, image_scan.EXIT_FINDINGS)
        self.assertIn("hit.png: tok:", out)
        self.assertNotIn("HX7Z", out)

    def test_undecodable_image_is_a_fault(self):
        with open(os.path.join(self.root, "bad.png"), "wb") as fh:
            fh.write(b"not an image")
        code, _ = run_main(["--patterns", self.patterns, "--root", self.root, "bad.png"])
        self.assertEqual(code, image_scan.EXIT_FAULT)


if __name__ == "__main__":
    unittest.main()
