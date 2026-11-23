#!/usr/bin/env python3
"""Refuse commits whose author or committer identity matches a leak pattern.

    author_scan.py --patterns FILE --head REV [--base REV] [--repo DIR]
    author_scan.py --self-test

The text scan in leak-check.yml reads tracked files only, so a commit's author and committer are
invisible to it. A merge or squash made on GitHub (the merge button, `gh pr merge`, the merges API)
is authored by GitHub with the account's commit email, and the pre-push hook never sees it, because
nobody pushed it. This reads every commit in BASE..HEAD (merges included; every commit reachable
from HEAD when --base is not given) and matches the same pattern document against the author name,
author email, committer name and committer email of each, separately. A finding is reported as
commit, field and pattern id only; the matched text is never printed, so the log does not
republish what it caught.

Commit messages are not read here.

It runs after the fact on a push: a merge GitHub made is already on the branch when this job turns
red. It cannot predict the identity GitHub will give a merge it has not made yet.

--self-test builds a throwaway repository with one commit carrying a canary author and one carrying
a canary committer between clean commits, scans it with a pattern that exists only for the test,
and requires exactly the planted fields to be found and nothing else. It is the instrument proving
it can fail before its silence is trusted.

Exit 0 clean, 1 findings, 2 usage, 3 instrument fault (git missing or failed, a revision that does
not resolve, more than MAX_COMMITS commits, or a self-test that did not behave). A fault is never
reported as clean.
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

MAX_COMMITS = 20_000
GIT_TIMEOUT_S = 120
FIELDS = ("author name", "author email", "committer name", "committer email")
FORMAT = "%H%x00%an%x00%ae%x00%cn%x00%ce"
SHA = re.compile(r"[0-9a-f]{40}")
CANARY_PATTERN = r"\bQK8V\b"
CANARY_NAME = "QK8V Canary"
CANARY_EMAIL = "qk8v@example.invalid"

EXIT_CLEAN, EXIT_FINDINGS, EXIT_USAGE, EXIT_FAULT = 0, 1, 2, 3


class Fault(Exception):
    """The instrument did not run correctly; never a statement about the commits."""


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


def git(repo, args, env=None):
    if shutil.which("git") is None:
        raise Fault("git is not installed")
    try:
        done = subprocess.run(["git", "-C", repo] + args, capture_output=True, timeout=GIT_TIMEOUT_S,
                              stdin=subprocess.DEVNULL, env=env)
    except subprocess.TimeoutExpired:
        raise Fault("git %s timed out" % args[0])
    if done.returncode != 0:
        raise Fault("git %s failed (rc %d)" % (args[0], done.returncode))
    return done.stdout.decode("utf-8", errors="replace")


def resolve(repo, rev):
    out = git(repo, ["rev-parse", "--verify", "--quiet", rev + "^{commit}"]).strip()
    if not SHA.fullmatch(out):
        raise Fault("revision does not resolve to a commit")
    return out


def commits(repo, base, head):
    """(sha, {field: text}) for every commit in base..head, or reachable from head without base."""
    span = [resolve(repo, head)] if base is None else ["%s..%s" % (resolve(repo, base), resolve(repo, head))]
    count = int(git(repo, ["rev-list", "--count"] + span).strip())
    if count > MAX_COMMITS:
        raise Fault("%d commits in range, over the %d limit" % (count, MAX_COMMITS))
    out = []
    for line in git(repo, ["log", "--no-color", "--format=" + FORMAT] + span).splitlines():
        if not line:
            continue
        parts = line.split("\x00")
        if len(parts) != 1 + len(FIELDS) or not SHA.fullmatch(parts[0]):
            raise Fault("unexpected git log record")
        out.append((parts[0], dict(zip(FIELDS, parts[1:]))))
    if len(out) != count:
        raise Fault("read %d commits, git counted %d" % (len(out), count))
    return out


def match_fields(fields, patterns):
    """[(field, pattern id)] for every field a pattern matches, in FIELDS order."""
    return [(field, pid) for field in FIELDS for pid, rx in patterns if rx.search(fields[field])]


def scan(repo, base, head, patterns):
    found = commits(repo, base, head)
    findings = 0
    for sha, fields in found:
        for field, pid in match_fields(fields, patterns):
            print("%s: %s: %s" % (sha[:12], field, pid))
            findings += 1
    print("scanned %d commit(s); %d finding(s)." % (len(found), findings))
    return found, findings


def self_test():
    work = tempfile.mkdtemp(prefix="author-scan-self-test-")
    try:
        repo = os.path.join(work, "repo")
        os.mkdir(repo)
        env = dict(os.environ, GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM="1")
        clean = dict(env, GIT_AUTHOR_NAME="Clean Author", GIT_AUTHOR_EMAIL="clean@example.invalid",
                     GIT_COMMITTER_NAME="Clean Author", GIT_COMMITTER_EMAIL="clean@example.invalid")
        git(repo, ["init", "-q"], env=clean)
        git(repo, ["commit", "-q", "--allow-empty", "-m", "Start"], env=clean)
        base = resolve(repo, "HEAD")
        git(repo, ["commit", "-q", "--allow-empty", "-m", "Add a file"],
            env=dict(clean, GIT_AUTHOR_NAME=CANARY_NAME, GIT_AUTHOR_EMAIL=CANARY_EMAIL))
        planted_author = resolve(repo, "HEAD")
        git(repo, ["commit", "-q", "--allow-empty", "-m", "Merge"], env=dict(clean, GIT_COMMITTER_EMAIL=CANARY_EMAIL))
        planted_committer = resolve(repo, "HEAD")
        git(repo, ["commit", "-q", "--allow-empty", "-m", "Tidy"], env=clean)
        patterns = [("canary", re.compile(CANARY_PATTERN, re.IGNORECASE))]
        got = [(sha, match_fields(fields, patterns)) for sha, fields in commits(repo, base, "HEAD")]
        want = {planted_author: [("author name", "canary"), ("author email", "canary")],
                planted_committer: [("committer email", "canary")]}
        if len(got) != 3 or {sha: hits for sha, hits in got if hits} != want:
            raise Fault("self-test: canary not found exactly as planted")
        if len(commits(repo, None, "HEAD")) != 4:
            raise Fault("self-test: a scan without --base did not read every commit")
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print("self-test passed: the planted author name, author email and committer email were found; "
          "the clean commits were not.")
    return EXIT_CLEAN


def parse(argv):
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--patterns")
    parser.add_argument("--head")
    parser.add_argument("--base")
    parser.add_argument("--repo", default=".")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    if not args.self_test and (not args.patterns or not args.head):
        parser.error("--patterns and --head are required unless --self-test")
    return args


def main(argv):
    try:
        args = parse(argv)
    except SystemExit as exc:
        return EXIT_USAGE if exc.code else EXIT_CLEAN
    try:
        if args.self_test:
            return self_test()
        _, findings = scan(args.repo, args.base, args.head, load_patterns(args.patterns))
    except Fault as exc:
        print("INSTRUMENT FAULT (not a finding): %s" % exc)
        return EXIT_FAULT
    return EXIT_FINDINGS if findings else EXIT_CLEAN


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
