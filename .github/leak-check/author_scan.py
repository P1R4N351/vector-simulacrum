#!/usr/bin/env python3
"""Refuse commits whose author or committer identity matches a leak pattern.

    author_scan.py --patterns FILE --head REV [--base REV] [--repo DIR]
    author_scan.py --self-test

The text scan in leak-check.yml reads tracked files only, so a commit's author and committer are
invisible to it. A merge or squash made on GitHub (the merge button, `gh pr merge`, the merges API)
is authored by GitHub with the account's commit email, and the pre-push hook never sees it, because
nobody pushed it. This reads every commit in BASE..HEAD (merges and both sides of them included;
every commit reachable from HEAD when --base is not given) and matches the same pattern document
against the author name, author email, committer name and committer email of each, separately. A
finding is reported as commit, field and pattern id only; the matched text is never printed, so the
log does not republish what it caught.

Commit messages are not read here. The co-author lines GitHub adds to a squash name the account's
noreply address, and one current pattern also matches the words of such a line, so a message scan
would add a false positive and no finding.

It runs after the fact on a push: a merge GitHub made is already on the branch when this job turns
red. It cannot predict the identity GitHub will give a merge it has not made yet.

--self-test builds a throwaway repository and scans it through the same path a real run takes,
pattern file and exit code included. It plants a canary in each of the four fields: in the middle
of a name, in lower case where the pattern is upper case with the i flag, and on a side branch
reachable only through a merge's second parent. It requires exactly those four findings, a clean
result on a clean range, and every commit read when no base is given. It is the instrument proving
it can fail before its silence is trusted.

Exit 0 clean, 1 findings, 2 usage, 3 instrument fault (a pattern file that is unreadable,
malformed or does not compile; git missing or failed; a revision that does not resolve; more than
MAX_COMMITS commits; or a self-test that did not behave). A fault is never reported as clean or as
a finding.
"""

import argparse
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from contextlib import redirect_stdout

MAX_COMMITS = 20_000
MAX_PATTERNS = 10_000
GIT_TIMEOUT_S = 120
FIELDS = ("author name", "author email", "committer name", "committer email")
FORMAT = "%H%x00%an%x00%ae%x00%cn%x00%ce"
SHA = re.compile(r"[0-9a-f]{40}")
FINDING = re.compile(r"[0-9a-f]{12}: .+: .+")
CANARY_DOC = {"patterns": [{"id": "canary", "category": "self-test", "regex": r"\bQK8V\b", "flags": "i"}]}
CANARY_EMAIL = "qk8v@example.invalid"

EXIT_CLEAN, EXIT_FINDINGS, EXIT_USAGE, EXIT_FAULT = 0, 1, 2, 3


class Fault(Exception):
    """The instrument did not run correctly; never a statement about the commits."""


def load_patterns(path):
    """[(id, compiled regex)] from a {"patterns": [{"id", "regex", "flags"}...]} document.

    Every way the document can be wrong is a Fault. A message names the pattern by position and
    id, never by its regex, since the regexes are the values the project keeps out of the log.
    """
    try:
        with open(path, encoding="utf-8") as fh:
            spec = json.load(fh)
    except (OSError, ValueError) as exc:
        raise Fault("cannot read pattern file: %s" % exc.__class__.__name__)
    entries = spec.get("patterns") if isinstance(spec, dict) else None
    if not isinstance(entries, list) or not entries:
        raise Fault("pattern file has no list of patterns")
    if len(entries) > MAX_PATTERNS:
        raise Fault("pattern file has %d patterns, over the %d limit" % (len(entries), MAX_PATTERNS))
    out = []
    for index, entry in enumerate(entries):
        if not (isinstance(entry, dict) and isinstance(entry.get("id"), str) and entry["id"]
                and isinstance(entry.get("regex"), str) and isinstance(entry.get("flags", ""), str)):
            raise Fault("pattern %d is not an object with a string id, regex and flags" % index)
        flags = re.IGNORECASE if "i" in entry.get("flags", "") else 0
        try:
            out.append((entry["id"], re.compile(entry["regex"], flags)))
        except (re.error, OverflowError, RecursionError):
            # re raises OverflowError for a repeat count it cannot hold and RecursionError for
            # deeply nested groups; neither is an re.error, and both mean the pattern does not compile.
            raise Fault("pattern %d (%s) does not compile" % (index, entry["id"]))
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


def run_scan(patterns_path, repo, base, head):
    """The whole of a real run after argument parsing: load, scan, choose the exit code."""
    _, findings = scan(repo, base, head, load_patterns(patterns_path))
    return EXIT_FINDINGS if findings else EXIT_CLEAN


def captured_scan(patterns_path, repo, base, head):
    out = io.StringIO()
    with redirect_stdout(out):
        code = run_scan(patterns_path, repo, base, head)
    return code, out.getvalue()


def commit(repo, env, message, **identity):
    git(repo, ["commit", "-q", "--allow-empty", "-m", message], env=dict(env, **identity))
    return resolve(repo, "HEAD")


def build_canary_repo(repo):
    """Plant one canary per field; return (base, last canary commit, {finding line})."""
    env = dict(os.environ, GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM="1",
               GIT_AUTHOR_NAME="Clean Author", GIT_AUTHOR_EMAIL="clean@example.invalid",
               GIT_COMMITTER_NAME="Clean Author", GIT_COMMITTER_EMAIL="clean@example.invalid")
    git(repo, ["init", "-q", "-b", "main"], env=env)
    base = commit(repo, env, "Start")
    git(repo, ["checkout", "-q", "-b", "side"], env=env)
    side = commit(repo, env, "Side work", GIT_AUTHOR_NAME="Ada QK8V Lovelace")
    git(repo, ["checkout", "-q", "main"], env=env)
    author_email = commit(repo, env, "Add a file", GIT_AUTHOR_EMAIL=CANARY_EMAIL)
    git(repo, ["merge", "-q", "--no-ff", "-m", "Merge side", "side"],
        env=dict(env, GIT_COMMITTER_NAME="Merge QK8V Bot"))
    merge = resolve(repo, "HEAD")
    committer_email = commit(repo, env, "Fix", GIT_COMMITTER_EMAIL=CANARY_EMAIL)
    commit(repo, env, "Tidy")
    want = {"%s: author name: canary" % side[:12], "%s: author email: canary" % author_email[:12],
            "%s: committer name: canary" % merge[:12], "%s: committer email: canary" % committer_email[:12]}
    return base, committer_email, want


def self_test():
    work = tempfile.mkdtemp(prefix="author-scan-self-test-")
    try:
        repo = os.path.join(work, "repo")
        os.mkdir(repo)
        patterns_path = os.path.join(work, "patterns.json")
        with open(patterns_path, "w", encoding="utf-8") as fh:
            json.dump(CANARY_DOC, fh)
        base, last_canary, want = build_canary_repo(repo)
        code, out = captured_scan(patterns_path, repo, base, "HEAD")
        got = {line for line in out.splitlines() if FINDING.fullmatch(line)}
        if code != EXIT_FINDINGS or got != want or "scanned 5 commit(s); 4 finding(s)." not in out:
            raise Fault("self-test: the canaries were not found exactly as planted")
        if "qk8v" in out.lower():
            raise Fault("self-test: the scan printed the text it matched")
        code, out = captured_scan(patterns_path, repo, last_canary, "HEAD")
        if code != EXIT_CLEAN or "scanned 1 commit(s); 0 finding(s)." not in out:
            raise Fault("self-test: a clean range was not reported clean")
        code, out = captured_scan(patterns_path, repo, None, "HEAD")
        if code != EXIT_FINDINGS or "scanned 6 commit(s); 4 finding(s)." not in out:
            raise Fault("self-test: a scan without --base did not read every commit")
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print("self-test passed: a canary in each of the four fields was found, through the pattern file, "
          "in the middle of a name, in lower case and behind a merge; the clean commits were not.")
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
        return run_scan(args.patterns, args.repo, args.base, args.head)
    except Fault as exc:
        print("INSTRUMENT FAULT (not a finding): %s" % exc)
        return EXIT_FAULT


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
