"""Tests for author_scan.py. They need git and nothing else.
Run: python3 -m unittest discover -s .github/leak-check -p 'test_author_scan.py' -v
"""

import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import author_scan  # noqa: E402

TOKEN = [("tok", re.compile(r"\bZQ4X\b", re.IGNORECASE))]
CLEAN = {"GIT_AUTHOR_NAME": "Clean", "GIT_AUTHOR_EMAIL": "clean@example.invalid",
         "GIT_COMMITTER_NAME": "Clean", "GIT_COMMITTER_EMAIL": "clean@example.invalid"}


def run_main(argv):
    out = io.StringIO()
    with redirect_stdout(out):
        code = author_scan.main(argv)
    return code, out.getvalue()


def fields(**overrides):
    base = {field: "" for field in author_scan.FIELDS}
    base.update({key.replace("_", " "): value for key, value in overrides.items()})
    return base


class Matching(unittest.TestCase):
    def test_each_field_is_matched_on_its_own(self):
        for field in author_scan.FIELDS:
            with self.subTest(field=field):
                self.assertEqual(author_scan.match_fields(fields(**{field.replace(" ", "_"): "x zq4x y"}), TOKEN),
                                 [(field, "tok")])

    def test_clean_fields(self):
        self.assertEqual(author_scan.match_fields(fields(author_email="a@example.invalid"), TOKEN), [])

    def test_commit_message_is_not_a_field(self):
        self.assertNotIn("message", author_scan.FIELDS)


class Repository(unittest.TestCase):
    def setUp(self):
        self.work = tempfile.mkdtemp(prefix="author-scan-test-")
        self.repo = os.path.join(self.work, "repo")
        os.mkdir(self.repo)
        self.env = dict(os.environ, GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM="1", **CLEAN)
        self.git("init", "-q")
        self.commit("Start")
        self.base = self.head()
        self.patterns = os.path.join(self.work, "patterns.json")
        with open(self.patterns, "w", encoding="utf-8") as fh:
            json.dump({"patterns": [{"id": "tok", "category": "test", "regex": r"\bZQ4X\b", "flags": "i"}]}, fh)

    def tearDown(self):
        shutil.rmtree(self.work, ignore_errors=True)

    def git(self, *args, env=None):
        subprocess.run(["git", "-C", self.repo] + list(args), check=True, capture_output=True,
                       env=env or self.env)

    def commit(self, message, **identity):
        self.git("commit", "-q", "--allow-empty", "-m", message, env=dict(self.env, **identity))

    def head(self):
        return subprocess.run(["git", "-C", self.repo, "rev-parse", "HEAD"], check=True, capture_output=True,
                              text=True).stdout.strip()

    def test_clean_range_exits_zero(self):
        self.commit("Tidy")
        code, out = run_main(["--patterns", self.patterns, "--repo", self.repo, "--base", self.base, "--head", "HEAD"])
        self.assertEqual(code, author_scan.EXIT_CLEAN, out)
        self.assertIn("scanned 1 commit(s); 0 finding(s).", out)

    def test_author_email_set_by_a_hosted_merge_is_found_without_printing_it(self):
        self.commit("Squash (#128)", GIT_AUTHOR_EMAIL="zq4x@example.invalid", GIT_COMMITTER_NAME="GitHub",
                    GIT_COMMITTER_EMAIL="noreply@github.com")
        sha = self.head()
        code, out = run_main(["--patterns", self.patterns, "--repo", self.repo, "--base", self.base, "--head", "HEAD"])
        self.assertEqual(code, author_scan.EXIT_FINDINGS, out)
        self.assertIn("%s: author email: tok" % sha[:12], out)
        self.assertNotIn("zq4x", out.lower())

    def test_committer_and_merge_commits_are_read(self):
        self.git("checkout", "-q", "-b", "side")
        self.commit("Side work")
        self.git("checkout", "-q", "-")
        self.git("merge", "-q", "--no-ff", "-m", "Merge side", "side",
                 env=dict(self.env, GIT_COMMITTER_EMAIL="zq4x@example.invalid"))
        code, out = run_main(["--patterns", self.patterns, "--repo", self.repo, "--base", self.base, "--head", "HEAD"])
        self.assertEqual(code, author_scan.EXIT_FINDINGS, out)
        self.assertIn("committer email: tok", out)
        self.assertIn("scanned 2 commit(s); 1 finding(s).", out)

    def test_commits_before_the_base_are_not_read(self):
        self.commit("Old", GIT_AUTHOR_EMAIL="zq4x@example.invalid")
        old = self.head()
        self.commit("New")
        code, out = run_main(["--patterns", self.patterns, "--repo", self.repo, "--base", old, "--head", "HEAD"])
        self.assertEqual(code, author_scan.EXIT_CLEAN, out)

    def test_without_a_base_every_reachable_commit_is_read(self):
        self.commit("Old", GIT_AUTHOR_EMAIL="zq4x@example.invalid")
        self.commit("New")
        code, out = run_main(["--patterns", self.patterns, "--repo", self.repo, "--head", "HEAD"])
        self.assertEqual(code, author_scan.EXIT_FINDINGS, out)
        self.assertIn("scanned 3 commit(s); 1 finding(s).", out)

    def test_unresolvable_revision_is_a_fault_not_clean(self):
        code, out = run_main(["--patterns", self.patterns, "--repo", self.repo, "--base", "0" * 40, "--head", "HEAD"])
        self.assertEqual(code, author_scan.EXIT_FAULT, out)
        self.assertIn("INSTRUMENT FAULT", out)

    def test_a_failing_git_command_is_a_fault(self):
        with self.assertRaises(author_scan.Fault):
            author_scan.git(self.repo, ["log", "no-such-revision"])

    def test_empty_pattern_file_is_a_fault(self):
        with open(self.patterns, "w", encoding="utf-8") as fh:
            json.dump({"patterns": []}, fh)
        code, out = run_main(["--patterns", self.patterns, "--repo", self.repo, "--head", "HEAD"])
        self.assertEqual(code, author_scan.EXIT_FAULT, out)

    def test_missing_head_is_usage(self):
        code, _ = run_main(["--patterns", self.patterns])
        self.assertEqual(code, author_scan.EXIT_USAGE)


class SelfTest(unittest.TestCase):
    def test_self_test_passes(self):
        code, out = run_main(["--self-test"])
        self.assertEqual(code, author_scan.EXIT_CLEAN, out)


if __name__ == "__main__":
    unittest.main()
