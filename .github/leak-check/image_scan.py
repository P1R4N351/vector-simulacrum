#!/usr/bin/env python3
"""OCR the given images and refuse any whose rendered text matches a leak pattern.

    image_scan.py --patterns FILE [--root DIR] [--list FILE] [PATH ...]
    image_scan.py --patterns FILE --self-test

The text scan in leak-check.yml skips binary files, so a name drawn into a plot legend, a title or
a photo caption is invisible to it. This reads the pixels instead: every frame (up to MAX_FRAMES)
is flattened onto white, converted to grey, enlarged SCALE times (plot legends are small), and
read by tesseract in sparse-text mode twice (grey, and thresholded), whose lines are matched against the same
pattern document the text scan uses. A finding is reported as path and pattern id only; the
matched text is never printed, so the log does not republish what it caught.

--list reads NUL-separated paths relative to --root (the output of `git diff --name-only -z`).
Paths that are not images by extension are ignored, so a raw diff list can be passed straight in.

--self-test renders a canary string into a small legend-sized image with Pillow, scans it with a
pattern that exists only for the test, and requires a finding; it also requires NO finding on a
second image without the canary. It is the instrument proving it can fail before its silence is
trusted (a check that cannot fail is not a check).

Exit 0 clean, 1 findings, 2 usage, 3 instrument fault (tesseract missing or failed, an image could
not be decoded, or the self-test did not behave). A fault is never reported as clean.
"""

import argparse
import io
import json
import os
import re
import shutil
import subprocess
import sys

IMAGE_EXTENSIONS = (".png", ".jpg", ".jpeg", ".gif", ".webp", ".bmp", ".tif", ".tiff")
MAX_IMAGES = 5000
MAX_FRAMES = 4
MAX_LIST_BYTES = 8_000_000
SCALE = 2
MAX_SCALED_PIXELS = 60_000_000
TESSERACT_TIMEOUT_S = 180
PAGE_SEG_MODE = "11"
BINARY_THRESHOLD = 128
CANARY_TEXT = "HX7Z canary 4W9"
CANARY_PATTERN = r"\bHX7Z\b"
FOLD = str.maketrans({"\u2018": "'", "\u2019": "'", "\u201c": '"', "\u201d": '"',
                      "\u2010": "-", "\u2011": "-", "\u2012": "-", "\u2013": "-",
                      "\u2014": "-", "\u2212": "-", "|": "I", "\u00a5": "Y"})
CASE_JOIN = re.compile(r"([a-z])([A-Z])")

EXIT_CLEAN, EXIT_FINDINGS, EXIT_USAGE, EXIT_FAULT = 0, 1, 2, 3


class Fault(Exception):
    """The instrument did not run correctly; never a statement about the image."""


def load_patterns(path):
    try:
        with open(path, encoding="utf-8") as fh:
            spec = json.load(fh)
    except (OSError, ValueError) as exc:
        raise Fault("cannot read pattern file: %s" % exc.__class__.__name__)
    out = []
    for entry in spec.get("patterns", []):
        flags = re.IGNORECASE if "i" in entry.get("flags", "") else 0
        out.append((entry["id"], re.compile(entry["regex"], flags)))
    if not out:
        raise Fault("pattern file has no patterns")
    return out


def frames_of(image, image_module):
    """Up to MAX_FRAMES frames, evenly spaced, each flattened onto white and converted to grey."""
    count = max(1, int(getattr(image, "n_frames", 1)))
    picks = sorted({(i * count) // MAX_FRAMES for i in range(min(count, MAX_FRAMES))})
    assert 1 <= len(picks) <= MAX_FRAMES
    out = []
    for index in picks:
        image.seek(index)
        rgba = image.convert("RGBA")
        canvas = image_module.new("RGBA", rgba.size, (255, 255, 255, 255))
        canvas.alpha_composite(rgba)
        grey = canvas.convert("L")
        if grey.width * grey.height * SCALE * SCALE <= MAX_SCALED_PIXELS:
            grey = grey.resize((grey.width * SCALE, grey.height * SCALE),
                               image_module.Resampling.LANCZOS)
        assert grey.mode == "L"
        out.append(grey)
    assert len(out) == len(picks)
    return out


def tesseract_lines(frame, tesseract):
    buffer = io.BytesIO()
    frame.save(buffer, format="PNG")
    try:
        proc = subprocess.run([tesseract, "stdin", "stdout", "--psm", PAGE_SEG_MODE, "-l", "eng"],
                              input=buffer.getvalue(), capture_output=True,
                              timeout=TESSERACT_TIMEOUT_S, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise Fault("tesseract did not complete (%s)" % exc.__class__.__name__)
    if proc.returncode != 0:
        raise Fault("tesseract exited %d" % proc.returncode)
    return proc.stdout.decode("utf-8", errors="replace").splitlines()


def ocr_frame(frame, tesseract):
    """Sparse-text OCR of the grey frame and of a hard-thresholded copy. The second pass reads
    light-on-dark photo captions whose anti-aliased glyphs the grey pass drops (measured: the
    grey pass lost a letter of a caption on a dark strip at every scale tried)."""
    binary = frame.point(lambda value: 255 if value > BINARY_THRESHOLD else 0)
    return tesseract_lines(frame, tesseract) + tesseract_lines(binary, tesseract)


def ocr_lines(path, tesseract, image_module):
    try:
        with image_module.open(path) as image:
            frames = frames_of(image, image_module)
    except (OSError, ValueError, image_module.DecompressionBombError) as exc:
        raise Fault("image could not be decoded (%s)" % exc.__class__.__name__)
    lines = []
    for frame in frames:
        lines.extend(ocr_frame(frame, tesseract))
    return lines


def variants(line):
    """The OCR line, a copy with typographic marks folded to ASCII, and a copy of that with a space
    restored at each lower-to-upper case join: tesseract drops the gap between a legend's words
    ("seriesNAME"), which defeats a pattern anchored on a word boundary."""
    folded = line.translate(FOLD)
    return (line, folded, CASE_JOIN.sub(r"\1 \2", folded))


def match_ids(lines, patterns):
    """Pattern ids that match any variant of any OCR line."""
    hits = []
    for line in lines:
        forms = variants(line)
        for pid, rx in patterns:
            if pid not in hits and any(rx.search(form) for form in forms):
                hits.append(pid)
    return hits


def read_list(path):
    try:
        with open(path, "rb") as fh:
            data = fh.read(MAX_LIST_BYTES + 1)
    except OSError as exc:
        raise Fault("cannot read path list: %s" % exc.__class__.__name__)
    if len(data) > MAX_LIST_BYTES:
        raise Fault("path list exceeds %d bytes" % MAX_LIST_BYTES)
    return [p for p in data.decode("utf-8", errors="strict").split("\0") if p]


def select_images(root, rels):
    chosen = []
    for rel in rels[:MAX_IMAGES + 1]:
        if rel.lower().endswith(IMAGE_EXTENSIONS) and os.path.isfile(os.path.join(root, rel)):
            chosen.append(rel)
    if len(chosen) > MAX_IMAGES:
        raise Fault("more than %d images to scan" % MAX_IMAGES)
    return chosen


def scan(root, rels, patterns, tesseract, image_module):
    findings = 0
    for rel in rels:
        for pid in match_ids(ocr_lines(os.path.join(root, rel), tesseract, image_module), patterns):
            print("%s: %s: (text rendered in the image; match not shown)" % (rel, pid))
            findings += 1
    print("ocr-scanned %d image(s); %d finding(s)." % (len(rels), findings))
    return EXIT_FINDINGS if findings else EXIT_CLEAN


def render(text, image_module, draw_module, font_module):
    """A legend-sized label: 10 px text on a 1x canvas, the size a matplotlib legend renders at."""
    font = font_module.load_default(size=10)
    image = image_module.new("RGB", (220, 40), (255, 255, 255))
    draw = draw_module.Draw(image)
    draw.line((8, 20, 30, 20), fill=(31, 119, 180), width=2)
    draw.text((36, 13), text, fill=(0, 0, 0), font=font)
    return image


def self_test(tesseract, modules):
    image_module, draw_module, font_module = modules
    patterns = [("ocr-canary", re.compile(CANARY_PATTERN))]
    results = {}
    for label, text in (("positive", "series " + CANARY_TEXT), ("negative", "series unit 3")):
        image = render(text, image_module, draw_module, font_module)
        lines = []
        for frame in frames_of(image, image_module):
            lines.extend(ocr_frame(frame, tesseract))
        results[label] = match_ids(lines, patterns)
    assert set(results) == {"positive", "negative"}
    if results["positive"] != ["ocr-canary"] or results["negative"]:
        raise Fault("self-test: canary found=%s, negative found=%s"
                    % (bool(results["positive"]), bool(results["negative"])))
    print("self-test: canary found in a legend-sized image, nothing found without it.")
    return EXIT_CLEAN


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--patterns", required=True)
    parser.add_argument("--root", default=".")
    parser.add_argument("--list", dest="list_file")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("paths", nargs="*")
    return parser.parse_args(argv)


def run(args):
    tesseract = shutil.which("tesseract")
    if not tesseract:
        raise Fault("tesseract is not installed")
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        raise Fault("Pillow is not installed")
    if args.self_test:
        return self_test(tesseract, (Image, ImageDraw, ImageFont))
    patterns = load_patterns(args.patterns)
    rels = list(args.paths) + (read_list(args.list_file) if args.list_file else [])
    return scan(args.root, select_images(args.root, rels), patterns, tesseract, Image)


def main(argv):
    try:
        args = parse_args(argv)
    except SystemExit as exc:
        return EXIT_USAGE if exc.code else EXIT_CLEAN
    try:
        return run(args)
    except Fault as exc:
        print("image-scan FAULT (the scan did not run; this is not a clean result): %s" % exc)
        return EXIT_FAULT


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
