#!/usr/bin/env python3
"""OCR the given images and refuse any whose rendered text matches a leak pattern.

    image_scan.py --patterns FILE [--root DIR] [--list FILE] [PATH ...]
    image_scan.py --patterns FILE --self-test

The text scan in leak-check.yml skips binary files, so a name drawn into a plot legend, a title or
a photo caption is invisible to it. This reads the pixels instead: every frame (up to MAX_FRAMES)
is flattened onto white and read by household_ocr.py, vendored beside this file and pinned by
household_ocr.sha256: the frame is upscaled 2x and read by tesseract 5.5 in sparse-text mode at 0, 90
and 270 degrees over three renderings (greyscale, dark text only, light text only), which catches
rotated axis labels, light text on dark panels and words beside a legend's line sample. Every line read
is matched against the same pattern document the text scan uses (with typographic marks folded and
case joins re-spaced), and short codes are also matched within one character (a misread such as KON7).
A finding is reported as path and pattern id only; the matched text is never printed, so the log does
not republish what it caught. OCR silence is not proof of absence: look at the images too.

--list reads NUL-separated paths relative to --root (the output of `git diff --name-only -z`).
Paths that are not images by extension are ignored, so a raw diff list can be passed straight in.

--self-test renders a canary string into small legend-sized images with Pillow (upright beside a line
sample, rotated as an axis label, and misread by one character), scans them with a pattern that exists
only for the test, and requires a finding in each; it also requires NO finding on an image without
the canary. It is the instrument proving it can fail before its silence is
trusted (a check that cannot fail is not a check).

Exit 0 clean, 1 findings, 2 usage, 3 instrument fault (tesseract missing, unpinned or failed, the
vendored library off its pin, an image that could not be decoded, or the self-test did not behave). A fault is never reported as clean.
"""

import argparse
import hashlib
import io
import json
import os
import re
import sys

IMAGE_EXTENSIONS = (".png", ".jpg", ".jpeg", ".gif", ".webp", ".bmp", ".tif", ".tiff")
MAX_IMAGES = 5000
MAX_FRAMES = 4
MAX_LIST_BYTES = 8_000_000
HERE = os.path.dirname(os.path.abspath(__file__))
CANARY_TEXT = "HX7Z canary 4W9"
CANARY_PATTERN = r"\bHX7Z\b"
FOLD = str.maketrans({"\u2018": "'", "\u2019": "'", "\u201c": '"', "\u201d": '"',
                      "\u2010": "-", "\u2011": "-", "\u2012": "-", "\u2013": "-",
                      "\u2014": "-", "\u2212": "-", "|": "I", "\u00a5": "Y"})
CASE_JOIN = re.compile(r"([a-z])([A-Z])")

EXIT_CLEAN, EXIT_FINDINGS, EXIT_USAGE, EXIT_FAULT = 0, 1, 2, 3


class Fault(Exception):
    """The instrument did not run correctly; never a statement about the image."""


def ocr_library():
    """The vendored household_ocr, refused when its bytes are off the pin."""
    try:
        with open(os.path.join(HERE, "household_ocr.sha256"), encoding="utf-8") as fh:
            want = fh.read().split()[0]
        with open(os.path.join(HERE, "household_ocr.py"), "rb") as fh:
            got = hashlib.sha256(fh.read()).hexdigest()
    except (OSError, IndexError) as exc:
        raise Fault("cannot read the vendored OCR library or its pin (%s)" % exc.__class__.__name__)
    if got != want:
        raise Fault("vendored household_ocr.py is %s, pinned %s" % (got[:16], want[:16]))
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    try:
        import household_ocr
    except ImportError as exc:
        raise Fault("household_ocr cannot load (%s); Pillow >= 10.1 is required" % exc.__class__.__name__)
    return household_ocr


def load_spec(path):
    try:
        with open(path, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError) as exc:
        raise Fault("cannot read pattern file: %s" % exc.__class__.__name__)


def load_patterns(path):
    spec = load_spec(path)
    out = []
    for entry in spec.get("patterns", []):
        flags = re.IGNORECASE if "i" in entry.get("flags", "") else 0
        out.append((entry["id"], re.compile(entry["regex"], flags)))
    if not out:
        raise Fault("pattern file has no patterns")
    return out


def frames_of(image, image_module):
    """Up to MAX_FRAMES frames, evenly spaced, each flattened onto white, as RGB at its own size (the
    OCR library upscales and uses colour itself)."""
    count = max(1, int(getattr(image, "n_frames", 1)))
    picks = sorted({(i * count) // MAX_FRAMES for i in range(min(count, MAX_FRAMES))})
    assert 1 <= len(picks) <= MAX_FRAMES
    out = []
    for index in picks:
        image.seek(index)
        rgba = image.convert("RGBA")
        canvas = image_module.new("RGBA", rgba.size, (255, 255, 255, 255))
        canvas.alpha_composite(rgba)
        out.append(canvas.convert("RGB"))
    assert len(out) == len(picks)
    return out


def frame_ids(frame, ocr, spec, patterns):
    """Pattern ids in one frame: this scanner's regex variants over every line the library read, plus
    the library's own matches (short codes within one character, regexes per line)."""
    buffer = io.BytesIO()
    frame.save(buffer, format="PNG")
    try:
        words = ocr.read_image(ocr.decode(buffer.getvalue()))
    except ocr.Fault as exc:
        raise Fault("OCR did not complete (%s)" % exc)
    ids = match_ids(ocr.joined_text(words).splitlines(), patterns)
    for hit in ocr.merge_hits(ocr.code_hits(words, spec) + ocr.regex_hits(words, spec)):
        if hit["pattern"] not in ids:
            ids.append(hit["pattern"])
    return ids


def ocr_ids(path, ocr, spec, patterns, image_module):
    try:
        with image_module.open(path) as image:
            frames = frames_of(image, image_module)
    except (OSError, ValueError, image_module.DecompressionBombError) as exc:
        raise Fault("image could not be decoded (%s)" % exc.__class__.__name__)
    ids = []
    for frame in frames:
        ids += [pid for pid in frame_ids(frame, ocr, spec, patterns) if pid not in ids]
    return ids


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


def scan(root, rels, patterns, ocr, spec, image_module):
    findings = 0
    for rel in rels:
        for pid in ocr_ids(os.path.join(root, rel), ocr, spec, patterns, image_module):
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


def self_test(ocr, modules):
    """Each canary must be found and the negative must not: upright beside a line sample, rotated as an
    axis label, and misread by one character."""
    image_module, draw_module, font_module = modules
    raw = [{"id": "ocr-canary", "regex": CANARY_PATTERN}]
    patterns, spec = [("ocr-canary", re.compile(CANARY_PATTERN))], ocr.compile_patterns(raw)
    cases = (("positive", "series " + CANARY_TEXT, 0, True), ("rotated", "range " + CANARY_TEXT, 90, True),
             ("misread", "series HX72 canary", 0, True), ("negative", "series unit 3", 0, False))
    wrong = []
    for label, text, angle, want in cases:
        image = render(text, image_module, draw_module, font_module)
        if angle:
            image = image.rotate(angle, expand=True, fillcolor=(255, 255, 255))
        found = frame_ids(image, ocr, spec, patterns) == ["ocr-canary"]
        if found != want:
            wrong.append("%s found=%s" % (label, found))
    if wrong:
        raise Fault("self-test: " + ", ".join(wrong))
    print("self-test: canaries found upright, rotated and misread; nothing found without the canary.")
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
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        raise Fault("Pillow is not installed")
    ocr = ocr_library()
    try:
        ocr.engine()
    except ocr.Fault as exc:
        raise Fault("OCR engine: %s" % exc)
    if args.self_test:
        return self_test(ocr, (Image, ImageDraw, ImageFont))
    patterns = load_patterns(args.patterns)
    try:
        spec = ocr.compile_patterns(load_spec(args.patterns))
    except ocr.Fault as exc:
        raise Fault("patterns: %s" % exc)
    rels = list(args.paths) + (read_list(args.list_file) if args.list_file else [])
    return scan(args.root, select_images(args.root, rels), patterns, ocr, spec, Image)


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
