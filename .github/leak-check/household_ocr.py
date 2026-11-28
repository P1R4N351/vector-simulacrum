#!/usr/bin/env python3
"""household_ocr: one OCR engine library. Spec: docs/HOUSEHOLD-OCR-DESIGN.md.

    household_ocr.py read IMAGE                         words, boxes, confidence, pass; joined text
    household_ocr.py find IMAGE --patterns P.json       hits as pattern id plus box
    household_ocr.py screen --patterns P.json IMAGE...  per image clean / findings / fault; no matched text
    household_ocr.py redact IMAGE --patterns P.json --replacements R.json --out OUT.png
    household_ocr.py version                            library, engine and traineddata pins

Engine: tesseract 5.5 with the version and the eng.traineddata sha256 pinned and checked before the
first read. Each image is upscaled 2x and read as sparse text with word boxes (--psm 11, tsv) at 0, 90
and 270 degrees over three renderings: greyscale, dark low-saturation pixels only (black text with
coloured plot lines removed) and light low-saturation pixels only (light text on dark). Boxes are mapped
back to original coordinates; overlapping boxes of the same pattern are merged.

Patterns (a JSON list, or an object with a "patterns" list): {"id", "regex", "flags"} matches each read
line; {"id", "codes": [...]} matches short codes fuzzily: an alphanumeric-reduced word, or a window of
it, within edit distance 1 of a code, the window holding at least one digit. A regex that is a plain
alternation of such codes (\\b(?:AB12|CD34)\\b) also gets the fuzzy match.

Every result is clean, findings or fault. A fault never reads as clean: an unreadable or oversize image,
an engine crash or timeout, a version or traineddata mismatch, or a pass that did not run. Images are
read from memory and handed to tesseract on stdin; nothing is written to disk. Exit: 0 clean, 1 findings
or unresolved, 2 usage, 3 fault.
"""

import argparse
import hashlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import warnings
from concurrent.futures import ThreadPoolExecutor

from PIL import Image, ImageDraw, ImageFont, ImageOps

VERSION = "1.2.0"
# P10: bounded. Every cap below is a hard limit; exceeding one is a fault, never a silent truncation.
TESSERACT_VERSIONS = ("5.5.0", "5.5.4")
ENG_TRAINEDDATA_SHA256 = "7d4322bd2a7749724879683fc3912cb542f19906c83bcc1a52132556427170b2"
TESSDATA_DIRS = ("/opt/homebrew/share/tessdata", "/usr/local/share/tessdata",
                 "/usr/share/tesseract-ocr/5/tessdata", "/usr/share/tessdata")
MAX_IMAGE_BYTES = 64 * 1024 * 1024
MAX_PIXELS = 16_000_000
MIN_SIDE = 8
MAX_IMAGES = 5000
MAX_PATTERNS = 256
MAX_CODES = 64
MAX_ROWS_PER_PASS = 50_000
MAX_WORD_CHARS = 64
MAX_HITS = 2000
MAX_REDACT_PASSES = 3
PASS_TIMEOUT_S = 300
SCALE = 2
ANGLES = (0, 90, 270)
RENDERINGS = ("grey", "dark-text", "light-text")
LAYOUTS = ("sparse", "document")
PSMS = ("11", "3", "6")
# The document layout is the liquidations label reader's measured recipe (2026-09-15): greyscale at the
# original size, upright, page modes 3 and 6, because mode 3 alone missed a brand that mode 6 read.
DOCUMENT_PASSES = (("grey@0/psm3", "3"), ("grey@0/psm6", "6"))
DARK_MAX_L = 110
LIGHT_MIN_L = 200
TEXT_MAX_S = 60
TRANSPARENT_BACKDROP = (128, 128, 128)
CODE_RE = re.compile(r"^[A-Za-z0-9]{4,12}$")
TSV_COLUMNS = 12
MAX_WIDEN = 1.15
MIN_FONT = 7
PAD = 2
_ENGINE = {}


class Fault(Exception):
    """An engine or input failure. Never reported as clean."""


def stamp():
    """The version stamp every result carries."""
    eng = engine()
    return {"library": VERSION, "engine": "tesseract " + eng["version"],
            "traineddata_sha256": ENG_TRAINEDDATA_SHA256}


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for _ in range(1 << 16):
            block = fh.read(1 << 20)
            if not block:
                return digest.hexdigest()
            digest.update(block)
    raise Fault("traineddata file larger than 64 GiB: %s" % path)


def find_tessdata():
    """The tessdata directory whose eng.traineddata matches the pinned sha256."""
    override = os.environ.get("HOUSEHOLD_OCR_TESSDATA", "")
    dirs = (override,) if override else TESSDATA_DIRS
    seen = []
    for d in dirs:
        path = os.path.join(d, "eng.traineddata")
        if os.path.isfile(path):
            got = sha256_file(path)
            if got == ENG_TRAINEDDATA_SHA256:
                return d
            seen.append("%s=%s" % (path, got[:16]))
    raise Fault("no eng.traineddata matching the pin (%s); seen: %s"
                % (ENG_TRAINEDDATA_SHA256[:16], ", ".join(seen) or "none"))


def engine():
    """Check the engine once per process: binary, exact version, traineddata hash."""
    if _ENGINE:
        return _ENGINE
    binary = shutil.which(os.environ.get("HOUSEHOLD_OCR_TESSERACT", "tesseract"))
    if binary is None:
        raise Fault("tesseract not found")
    proc = subprocess.run([binary, "--version"], capture_output=True, timeout=60, check=False)
    if proc.returncode != 0:
        raise Fault("tesseract --version exited %d" % proc.returncode)
    first = (proc.stdout + proc.stderr).decode("utf-8", "replace").splitlines()[:1]
    version = first[0].split()[-1] if first and first[0].startswith("tesseract ") else ""
    if version not in TESSERACT_VERSIONS:
        raise Fault("tesseract version %r is not pinned (%s)" % (version, ", ".join(TESSERACT_VERSIONS)))
    _ENGINE.update(binary=binary, version=version, tessdata=find_tessdata())
    return _ENGINE


def decode(data):
    """Bytes to an RGB image, within the byte and pixel caps. Transparency goes onto mid grey so that
    dark and light text both keep their contrast."""
    if not isinstance(data, (bytes, bytearray)) or not data:
        raise Fault("image is empty or not bytes")
    if len(data) > MAX_IMAGE_BYTES:
        raise Fault("image is %d bytes; the cap is %d" % (len(data), MAX_IMAGE_BYTES))
    Image.MAX_IMAGE_PIXELS = MAX_PIXELS
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("error", Image.DecompressionBombWarning)
            img = Image.open(io.BytesIO(bytes(data)))
        img.seek(0)
        if img.width * img.height > MAX_PIXELS:
            raise Fault("image is %dx%d; the cap is %d pixels" % (img.width, img.height, MAX_PIXELS))
        img.load()
    except (OSError, ValueError, SyntaxError, Image.DecompressionBombError,
            Image.DecompressionBombWarning) as exc:
        raise Fault("cannot decode image: %s" % type(exc).__name__)
    if min(img.size) < MIN_SIDE:
        raise Fault("image is %dx%d; too small to read" % img.size)
    try:
        img = ImageOps.exif_transpose(img)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        raise Fault("cannot apply the EXIF orientation: %s" % type(exc).__name__)
    if img.mode in ("RGBA", "LA", "PA") or (img.mode == "P" and "transparency" in img.info):
        rgba = img.convert("RGBA")
        backdrop = Image.new("RGBA", rgba.size, TRANSPARENT_BACKDROP + (255,))
        return Image.alpha_composite(backdrop, rgba).convert("RGB")
    return img.convert("RGB")


def renderings(img):
    """The 2x image as (name, black-on-white L image) for each rendering, in RENDERINGS order."""
    big = img.resize((img.width * SCALE, img.height * SCALE), Image.LANCZOS)
    grey = big.convert("L")
    low_sat = big.convert("HSV").getchannel("S").point([255 if v < TEXT_MAX_S else 0 for v in range(256)])
    dark = Image.composite(grey.point([0 if v < DARK_MAX_L else 255 for v in range(256)]),
                           Image.new("L", grey.size, 255), low_sat)
    light = Image.composite(grey.point([0 if v > LIGHT_MIN_L else 255 for v in range(256)]),
                            Image.new("L", grey.size, 255), low_sat)
    out = {"grey": grey, "dark-text": dark, "light-text": light}
    return [(name, out[name]) for name in RENDERINGS]


def to_original(box, angle, big_size, scale=SCALE):
    """A (l, t, w, h) box read on the rotated, scaled image, in original-image coordinates."""
    assert angle in ANGLES, angle
    l, t, w, h = box
    bw, bh = big_size
    if angle == 90:
        l, t, w, h = bw - (t + h), l, h, w
    elif angle == 270:
        l, t, w, h = t, bh - (l + w), h, w
    left, top = l // scale, t // scale
    right, bottom = -(-(l + w) // scale), -(-(t + h) // scale)
    return [left, top, max(1, right - left), max(1, bottom - top)]


def run_tesseract(png, psm="11"):
    """tesseract on stdin, tsv on stdout; any failure is a Fault."""
    assert psm in PSMS, psm
    eng = engine()
    argv = [eng["binary"], "stdin", "stdout", "--tessdata-dir", eng["tessdata"], "-l", "eng",
            "--psm", psm, "tsv"]
    env = dict(os.environ, OMP_THREAD_LIMIT="1")
    try:
        proc = subprocess.run(argv, input=png, capture_output=True, timeout=PASS_TIMEOUT_S, env=env,
                              check=False)
    except subprocess.TimeoutExpired:
        raise Fault("tesseract timed out after %ds" % PASS_TIMEOUT_S)
    if proc.returncode != 0:
        raise Fault("tesseract exited %d" % proc.returncode)
    text = proc.stdout.decode("utf-8", "replace")
    if not text.startswith("level\t"):
        raise Fault("tesseract printed no tsv header")
    return text


def parse_tsv(text, angle, big_size, label, scale=SCALE):
    """Words of one pass: text, original-coordinate box, confidence, pass label and line key."""
    rows = text.splitlines()[1:]
    if len(rows) > MAX_ROWS_PER_PASS:
        raise Fault("pass %s read %d rows; the cap is %d" % (label, len(rows), MAX_ROWS_PER_PASS))
    words = []
    for row in rows:
        cols = row.split("\t")
        if len(cols) != TSV_COLUMNS or not cols[11].strip():
            continue
        box = [int(c) for c in cols[6:10]]
        words.append({"text": cols[11].strip()[:MAX_WORD_CHARS],
                      "box": to_original(box, angle, big_size, scale),
                      "conf": round(float(cols[10]), 2), "pass": label,
                      "line": (label, int(cols[2]), int(cols[3]), int(cols[4]))})
    return words


def one_pass(job):
    """Read one image at one angle and page mode. Returns (label, words)."""
    label, image, angle, psm, scale = job
    rotated = image.rotate(angle, expand=True) if angle else image
    buf = io.BytesIO()
    rotated.save(buf, "PNG", compress_level=1)
    return label, parse_tsv(run_tesseract(buf.getvalue(), psm), angle, image.size, label, scale)


def expected_passes(layout):
    """The pass labels a layout must run, derived from the constants, not from the jobs built."""
    if layout == "document":
        return [label for label, _ in DOCUMENT_PASSES]
    return ["%s@%d" % (name, angle) for name in RENDERINGS for angle in ANGLES]


def jobs_for(img, layout):
    if layout == "document":
        grey = img.convert("L")
        return [(label, grey, 0, psm, 1) for label, psm in DOCUMENT_PASSES]
    return [("%s@%d" % (name, angle), image, angle, "11", SCALE) for name, image in renderings(img) for angle in ANGLES]


def read_image(img, workers=3, layout="sparse"):
    """All passes of a layout over a decoded image, in fixed pass order. A pass that did not run is a Fault."""
    if layout not in LAYOUTS:
        raise Fault("layout must be one of %s" % ", ".join(LAYOUTS))
    jobs = jobs_for(img, layout)
    expected = expected_passes(layout)
    with ThreadPoolExecutor(max_workers=max(1, min(workers, len(jobs)))) as pool:
        results = list(pool.map(one_pass, jobs))
    if [label for label, _ in results] != expected:
        raise Fault("passes run %r, expected %r" % ([label for label, _ in results], expected))
    return [word for _, words in results for word in words]


def joined_text(words):
    """Lines in pass order, words in reading order within a line."""
    lines = {}
    for word in words:
        lines.setdefault(word["line"], []).append(word["text"])
    return "\n".join(" ".join(parts) for parts in lines.values())


def text_by_pass(words, layout="sparse"):
    """{pass label: that pass's joined lines}, in pass order, so a caller can keep the upright reads."""
    out = {}
    for label in expected_passes(layout):
        out[label] = joined_text([w for w in words if w["pass"] == label])
    return out


def read(data, workers=3, layout="sparse"):
    """Words with boxes, confidence and the pass that saw each, plus joined text. layout "sparse" is the
    9-pass screening read; "document" reads dense text (labels, pages) upright at page modes 3 and 6."""
    try:
        words = read_image(decode(data), workers, layout)
    except Fault as exc:
        return {"status": "fault", "fault": str(exc), "version": VERSION}
    public = [{k: w[k] for k in ("text", "box", "conf", "pass")} for w in words]
    return {"status": "ok", "layout": layout, "words": public, "text": joined_text(words),
            "text_by_pass": text_by_pass(words, layout), "sha256": sha(data), "stamp": stamp()}


def sha(data):
    return hashlib.sha256(bytes(data)).hexdigest()


def derive_codes(regex):
    """The codes of a regex that is a plain alternation of short alphanumeric codes, else ()."""
    core = regex.replace("\\b", "").replace("(?:", "").replace("(", "").replace(")", "")
    parts = core.split("|")
    if not parts or len(parts) > MAX_CODES:
        return ()
    if all(CODE_RE.match(p) and any(c.isdigit() for c in p) for p in parts):
        return tuple(p.upper() for p in parts)
    return ()


def compile_patterns(spec):
    """[(id, compiled regex or None, codes)] from a pattern list or {"patterns": [...]}."""
    entries = spec.get("patterns", []) if isinstance(spec, dict) else spec
    if not isinstance(entries, list) or not entries or len(entries) > MAX_PATTERNS:
        raise Fault("patterns must be a non-empty list of at most %d entries" % MAX_PATTERNS)
    out = []
    for entry in entries:
        regex = entry.get("regex") or ""
        flags = re.IGNORECASE if "i" in entry.get("flags", "") else 0
        try:
            compiled = re.compile(regex, flags) if regex else None
        except re.error as exc:
            raise Fault("pattern %r does not compile: %s" % (entry.get("id"), exc))
        codes = tuple(str(c).upper() for c in entry.get("codes", [])[:MAX_CODES]) or derive_codes(regex)
        if not entry.get("id") or (compiled is None and not codes):
            raise Fault("pattern entry needs an id and a regex or codes")
        out.append((str(entry["id"]), compiled, codes))
    return out


def distance(a, b):
    """Levenshtein distance; both strings are at most MAX_WORD_CHARS long."""
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a[:MAX_WORD_CHARS], 1):
        cur = [i]
        for j, cb in enumerate(b[:MAX_WORD_CHARS], 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


def reduce_word(text):
    return "".join(c for c in text.upper() if c.isascii() and c.isalnum())[:MAX_WORD_CHARS]


def code_index(text, codes):
    """Index of the first code the word matches fuzzily, else -1."""
    word = reduce_word(text)
    for index, code in enumerate(codes):
        if len(word) < len(code) - 1:
            continue
        sizes = (len(code), len(code) + 1)
        windows = {word} | {word[i:i + n] for n in sizes for i in range(max(0, len(word) - n + 1))}
        for window in sorted(windows):
            if any(c.isdigit() for c in window) and distance(window, code) <= 1:
                return index
    return -1


def union(a, b):
    left, top = min(a[0], b[0]), min(a[1], b[1])
    right, bottom = max(a[0] + a[2], b[0] + b[2]), max(a[1] + a[3], b[1] + b[3])
    return [left, top, right - left, bottom - top]


def overlap(a, b):
    return a[0] < b[0] + b[2] and b[0] < a[0] + a[2] and a[1] < b[1] + b[3] and b[1] < a[1] + a[3]


def angle_of(label):
    return int(label.split("@")[1])


def code_hit(pid, index, codes, parts):
    """A short-code hit over one word, or two adjacent words of a line read as one."""
    box = parts[0]["box"] if len(parts) == 1 else union(parts[0]["box"], parts[1]["box"])
    return {"pattern": pid, "code": index, "codes": codes, "box": list(box),
            "angle": angle_of(parts[0]["pass"]), "conf": min(p["conf"] for p in parts),
            "pass": parts[0]["pass"], "word": "".join(p["text"] for p in parts)}


def code_hits(words, patterns):
    """Fuzzy short-code hits per word, and per adjacent word pair of a line where neither word hit
    alone: OCR splits a code across two words ("AB1" + "2cd") as readily as it misreads one letter."""
    hits = []
    for _, spans in line_spans(words).values():
        line = [w for _, _, w in spans]
        for pid, _, codes in patterns:
            alone = [code_index(w["text"], codes) if codes else -1 for w in line]
            hits += [code_hit(pid, i, codes, [w]) for w, i in zip(line, alone) if i >= 0]
            for k in range(len(line) - 1):
                if codes and alone[k] < 0 and alone[k + 1] < 0:
                    index = code_index(line[k]["text"] + line[k + 1]["text"], codes)
                    if index >= 0:
                        hits.append(code_hit(pid, index, codes, line[k:k + 2]))
    return hits


def line_spans(words):
    """{line key: (text, [(start, end, word)])} with words joined by single spaces."""
    lines = {}
    for word in words:
        text, spans = lines.get(word["line"], ("", []))
        start = len(text) + (1 if text else 0)
        text = (text + " " if text else "") + word["text"]
        lines[word["line"]] = (text, spans + [(start, len(text), word)])
    return lines


def regex_hits(words, patterns):
    """Regex hits over each read line; the box is the union of the words the match touches."""
    hits = []
    for text, spans in line_spans(words).values():
        for pid, compiled, _ in patterns:
            for match in (compiled.finditer(text) if compiled else ()):
                touched = [w for s, e, w in spans if s < match.end() and match.start() < e]
                if not touched:
                    continue
                box = touched[0]["box"]
                for w in touched[1:]:
                    box = union(box, w["box"])
                hits.append({"pattern": pid, "code": -1, "box": box, "angle": angle_of(touched[0]["pass"]),
                             "conf": max(w["conf"] for w in touched), "pass": touched[0]["pass"],
                             "word": " ".join(w["text"] for w in touched)})
    return hits


def merge_hits(hits):
    """Overlapping boxes of the same pattern merge into their union. Highest confidence first, so the
    kept angle, code and word come from the best read; ties break on the pass label for determinism."""
    ordered = sorted(hits, key=lambda h: (-h["conf"], h["pass"], h["box"]))[:MAX_HITS]
    merged = []
    for hit in ordered:
        for kept in merged:
            if kept["pattern"] == hit["pattern"] and overlap(kept["box"], hit["box"]):
                kept["box"] = union(kept["box"], hit["box"])
                kept["seen"] = kept["seen"] + 1
                break
        else:
            merged.append(dict(hit, seen=1))
    return merged


def hits_in(img, patterns, workers=3):
    """Merged hits (with the read word, for redaction) over every pass of a decoded image."""
    words = read_image(img, workers)
    return merge_hits(code_hits(words, patterns) + regex_hits(words, patterns))


def public_hit(hit):
    return {"pattern": hit["pattern"], "box": hit["box"], "angle": hit["angle"]}


def find(data, patterns, workers=3):
    """Hits as pattern id plus box (and the angle it read at). status clean, findings or fault."""
    try:
        compiled = compile_patterns(patterns)
        hits = hits_in(decode(data), compiled, workers)
    except Fault as exc:
        return {"status": "fault", "fault": str(exc), "version": VERSION}
    return {"status": "findings" if hits else "clean", "hits": [public_hit(h) for h in hits],
            "sha256": sha(data), "stamp": stamp()}


def screen(images, patterns, workers=3):
    """images: [(label, bytes)]. Per image clean, findings (pattern id and box) or fault. Never returns
    matched text. One faulted image faults the verdict; the others' results are still returned."""
    if not isinstance(images, list) or not images or len(images) > MAX_IMAGES:
        return {"verdict": "fault", "fault": "images must be a list of 1..%d" % MAX_IMAGES, "results": []}
    results = []
    for label, data in images:
        out = find(data, patterns, workers)
        row = {"image": label, "status": out["status"]}
        if out["status"] == "fault":
            row["fault"] = out["fault"]
        else:
            row["sha256"] = out["sha256"]
            row["findings"] = [{"pattern": h["pattern"], "box": h["box"]} for h in out["hits"]]
        results.append(row)
    statuses = {r["status"] for r in results}
    verdict = "fault" if "fault" in statuses else "findings" if "findings" in statuses else "clean"
    return {"verdict": verdict, "results": results, "stamp": stamp() if verdict != "fault" else
            {"library": VERSION}}


def font(size):
    """Pillow's built-in scalable font, so every host draws the same glyphs. No FreeType is a Fault."""
    face = ImageFont.load_default(size=size)
    if not isinstance(face, ImageFont.FreeTypeFont):
        raise Fault("Pillow has no FreeType; cannot draw replacements")
    return face


def cap_ratio():
    """Cap height of the drawing font as a fraction of its size, measured, not assumed."""
    if "cap" not in _ENGINE:
        probe = ImageDraw.Draw(Image.new("L", (1, 1)))
        _, top, _, bottom = probe.textbbox((0, 0), "H", font=font(100))
        _ENGINE["cap"] = max(0.3, (bottom - top) / 100.0)
    return _ENGINE["cap"]


def border_colour(img, box):
    """Median colour, per channel, of the ring just outside the box."""
    w, h = img.size
    left, top = max(0, box[0] - PAD), max(0, box[1] - PAD)
    right, bottom = min(w - 1, box[0] + box[2] + PAD), min(h - 1, box[1] + box[3] + PAD)
    pts = [img.getpixel((x, y)) for x in range(left, right + 1) for y in (top, bottom)]
    pts += [img.getpixel((x, y)) for y in range(top, bottom + 1) for x in (left, right)]
    return tuple(sorted(p[i] for p in pts)[len(pts) // 2] for i in range(3))


def replacement_text(hit, replacements):
    """The replacement for a hit: per pattern id a string, or a list aligned with the pattern's codes.
    A short-code hit keeps the read word's punctuation and suffix; one stray letter or digit glued to
    the code is an OCR fragment and is dropped."""
    value = replacements.get(hit["pattern"], "")
    if isinstance(value, list):
        value = value[hit["code"]] if 0 <= hit["code"] < len(value) else (value[0] if value else "")
    word = hit.get("word", "").replace("|", "")
    code = hit.get("codes", ("",))[hit["code"]] if hit["code"] >= 0 and hit.get("codes") else ""
    if not code or not word:
        return str(value)
    best = None
    for n in (len(code) - 1, len(code), len(code) + 1):
        for i in range(max(0, len(word) - n + 1)):
            d = distance(word[i:i + n].upper(), code)
            if best is None or d < best[0]:
                best = (d, i, n)
    if best is None or best[0] > 1:
        return str(value)
    _, i, n = best
    head, tail = word[:i], word[i + n:]
    head = "" if len(head) == 1 and head.isalnum() else head
    tail = "" if len(tail) == 1 and tail.isalnum() else tail
    return head + str(value) + tail


def fit_font(text, along, across):
    """A font whose cap height matches the box, shrunk until the text fits MAX_WIDEN of its length."""
    size = max(MIN_FONT, int(round(max(1, across - 1) / cap_ratio())))
    probe = ImageDraw.Draw(Image.new("L", (1, 1)))
    for _ in range(256):
        if size <= MIN_FONT or probe.textlength(text, font=font(size)) <= along * MAX_WIDEN:
            break
        size -= 1
    return font(size)


def paint(img, hit, text):
    """Cover the box with its border colour and draw text at cap height, rotated like the original."""
    box, angle = hit["box"], hit["angle"]
    bg = border_colour(img, box)
    ImageDraw.Draw(img).rectangle([box[0] - PAD, box[1] - PAD, box[0] + box[2] + PAD,
                                   box[1] + box[3] + PAD], fill=bg)
    ink = (0, 0, 0, 255) if 0.299 * bg[0] + 0.587 * bg[1] + 0.114 * bg[2] > 128 else (255, 255, 255, 255)
    along, across = (box[2], box[3]) if angle == 0 else (box[3], box[2])
    face = fit_font(text, along, across)
    probe = ImageDraw.Draw(Image.new("RGBA", (1, 1)))
    l, t, r, b = probe.textbbox((0, 0), text, font=face)
    label = Image.new("RGBA", (max(1, r - l + 2), max(1, b - t + 2)), (0, 0, 0, 0))
    ImageDraw.Draw(label).text((1 - l, 1 - t), text, fill=ink, font=face)
    if angle:
        label = label.rotate(-angle, expand=True)
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    cx, cy = box[0] + box[2] // 2, box[1] + box[3] // 2
    layer.paste(label, (cx - label.size[0] // 2, cy - label.size[1] // 2))
    return Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB"), list(bg)


def redact_pass(img, hits, replacements, number):
    edits = []
    for hit in hits:
        text = replacement_text(hit, replacements)
        img, fill = paint(img, hit, text)
        edits.append({"pass": number, "pattern": hit["pattern"], "box": hit["box"], "angle": hit["angle"],
                      "fill": fill, "replacement": text})
    return img, edits


def redact(data, patterns, replacements, hits=None, workers=3):
    """Paint every hit and redraw its replacement; repeat until find is empty (clean) or, after
    MAX_REDACT_PASSES, report unresolved. hits, if given, are a prior find's hits for the first pass.
    Returns (png bytes or None, result)."""
    try:
        compiled = compile_patterns(patterns)
        img = decode(data)
        current = [dict(h, code=-1, conf=0.0) for h in hits] if hits else None
        edits = []
        for number in range(1, MAX_REDACT_PASSES + 1):
            current = current if current is not None else hits_in(img, compiled, workers)
            if not current:
                return encode(img), {"status": "clean", "edits": edits, "passes": number - 1,
                                     "sha256": sha(data), "stamp": stamp()}
            img, done = redact_pass(img, current, replacements, number)
            edits += done
            current = None
        left = hits_in(img, compiled, workers)
    except Fault as exc:
        return None, {"status": "fault", "fault": str(exc), "version": VERSION}
    status = "unresolved" if left else "clean"
    return encode(img), {"status": status, "edits": edits, "passes": MAX_REDACT_PASSES,
                         "remaining": [public_hit(h) for h in left], "sha256": sha(data), "stamp": stamp()}


def encode(img):
    buf = io.BytesIO()
    img.save(buf, "PNG", optimize=True)
    return buf.getvalue()


def load_json(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def load_bytes(path):
    size = os.path.getsize(path)
    if size > MAX_IMAGE_BYTES:
        raise Fault("%s is %d bytes; the cap is %d" % (path, size, MAX_IMAGE_BYTES))
    with open(path, "rb") as fh:
        return fh.read()


def exit_code(status):
    return {"clean": 0, "ok": 0, "findings": 1, "unresolved": 1}.get(status, 3)


def cli_redact(args):
    data = load_bytes(args.image)
    png, result = redact(data, load_json(args.patterns), load_json(args.replacements), workers=args.workers)
    if png is not None:
        with open(args.out, "wb") as fh:
            fh.write(png)
    print(json.dumps(result, indent=1))
    return exit_code(result["status"])


def cli(args):
    if args.verb == "version":
        print(json.dumps(stamp(), indent=1))
        return 0
    if args.verb == "read":
        result = read(load_bytes(args.image), args.workers, args.layout)
    elif args.verb == "find":
        result = find(load_bytes(args.image), load_json(args.patterns), args.workers)
    elif args.verb == "screen":
        result = screen([(p, load_bytes(p)) for p in args.images[:MAX_IMAGES]], load_json(args.patterns),
                        args.workers)
        print(json.dumps(result, indent=1))
        return exit_code(result["verdict"])
    else:
        return cli_redact(args)
    print(json.dumps(result, indent=1))
    return exit_code(result["status"])


def parser():
    top = argparse.ArgumentParser(description="household OCR: read, find, screen, redact")
    top.add_argument("--workers", type=int, default=3, help="passes run in parallel (default 3)")
    sub = top.add_subparsers(dest="verb", required=True)
    sub.add_parser("version")
    rd = sub.add_parser("read")
    rd.add_argument("image")
    rd.add_argument("--layout", choices=LAYOUTS, default="sparse")
    f = sub.add_parser("find")
    f.add_argument("image")
    f.add_argument("--patterns", required=True)
    s = sub.add_parser("screen")
    s.add_argument("--patterns", required=True)
    s.add_argument("images", nargs="+")
    r = sub.add_parser("redact")
    r.add_argument("image")
    r.add_argument("--patterns", required=True)
    r.add_argument("--replacements", required=True)
    r.add_argument("--out", required=True)
    return top


def main(argv):
    args = parser().parse_args(argv)
    try:
        return cli(args)
    except (Fault, OSError, ValueError) as exc:
        print(json.dumps({"status": "fault", "fault": str(exc), "version": VERSION}), file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
