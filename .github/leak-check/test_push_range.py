"""Tests for push_range.sh. They need git and bash and nothing else.
Run: python3 -m unittest discover -s .github/leak-check -p 'test_push_range.py' -v

Each case pushes to a bare repository standing in for GitHub, then builds a runner clone the way
actions/checkout with fetch-depth 0 does (branch tips and tags only, HEAD detached at the pushed
commit) and runs the script there with the event's variables.
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "push_range.sh")
ZERO = "0" * 40
MAIN = "refs/heads/main"
sys.path.insert(0, HERE)
import author_scan  # noqa: E402

IDENTITY = {"GIT_AUTHOR_NAME": "Clean", "GIT_AUTHOR_EMAIL": "clean@example.invalid",
            "GIT_COMMITTER_NAME": "Clean", "GIT_COMMITTER_EMAIL": "clean@example.invalid"}


class PushRange(unittest.TestCase):
    def setUp(self):
        self.work = tempfile.mkdtemp(prefix="push-range-test-")
        self.env = dict(os.environ, GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM="1", **IDENTITY)
        self.dev = os.path.join(self.work, "dev")
        self.remote = os.path.join(self.work, "remote.git")
        self.git(self.work, "init", "-q", "-b", "main", self.dev)
        self.git(self.work, "init", "-q", "--bare", self.remote)
        self.runs = 0

    def tearDown(self):
        shutil.rmtree(self.work, ignore_errors=True)

    def git(self, repo, *args, env=None):
        done = subprocess.run(["git", "-C", repo] + list(args), check=True, capture_output=True, text=True,
                              env=env or self.env)
        return done.stdout.strip()

    def commit(self, message, **identity):
        self.git(self.dev, "commit", "-q", "--allow-empty", "-m", message, env=dict(self.env, **identity))
        return self.git(self.dev, "rev-parse", "HEAD")

    def push(self, sha, ref, force=False):
        self.git(self.dev, "push", "-q", "-f" if force else "--no-force", self.remote, "%s:%s" % (sha, ref))

    def drop_unreferenced(self):
        # The server no longer holds the old tip, so nothing can fetch it.
        self.git(self.remote, "reflog", "expire", "--expire=now", "--all")
        self.git(self.remote, "gc", "-q", "--prune=now")

    def runner_at(self, sha):
        self.runs += 1
        runner = os.path.join(self.work, "runner%d" % self.runs)
        self.git(self.work, "init", "-q", runner)
        self.git(runner, "remote", "add", "origin", "file://" + self.remote)
        self.git(runner, "fetch", "-q", "--prune", "origin",
                 "+refs/heads/*:refs/remotes/origin/*", "+refs/tags/*:refs/tags/*")
        self.git(runner, "fetch", "-q", "origin", sha)
        self.git(runner, "-c", "advice.detachedHead=false", "checkout", "-q", "--detach", sha)
        return runner

    def run_script(self, runner, event, before="", ref=MAIN):
        output = os.path.join(self.work, "output%d" % self.runs)
        open(output, "w").close()
        env = dict(self.env, EVENT=event, BEFORE=before, REF=ref, GITHUB_OUTPUT=output)
        done = subprocess.run(["bash", SCRIPT], cwd=runner, capture_output=True, text=True, env=env)
        with open(output, encoding="utf-8") as fh:
            values = dict(line.split("=", 1) for line in fh.read().splitlines() if "=" in line)
        return done.returncode, done.stdout + done.stderr, values.get("base"), values.get("head")

    def test_push_reads_from_the_previous_tip(self):
        self.commit("one")
        old = self.commit("two")
        self.push(old, MAIN)
        new = self.commit("three")
        self.push(new, MAIN)
        code, out, base, head = self.run_script(self.runner_at(new), "push", old)
        self.assertEqual((code, base, head), (0, old, new), out)
        self.assertIn("1 commit(s).", out)

    def test_force_push_to_main_with_the_old_tip_gone_is_a_fault(self):
        self.commit("one")
        old = self.commit("two", GIT_AUTHOR_EMAIL="old@example.invalid")
        self.push(old, MAIN)
        self.git(self.dev, "reset", "-q", "--hard", "HEAD^")
        new = self.commit("rewrite", GIT_AUTHOR_EMAIL="zq4x@example.invalid")
        self.push(new, MAIN, force=True)
        self.drop_unreferenced()
        code, out, base, head = self.run_script(self.runner_at(new), "push", old)
        self.assertEqual(code, 3, out)
        self.assertIn("::error::INSTRUMENT FAULT", out)
        self.assertIsNone(base, out)

    def test_force_push_to_main_fetches_an_old_tip_the_server_still_serves(self):
        # The checkout has branch tips only, so the old tip is absent until the script fetches it
        # by id from a server that still holds it.
        start = self.commit("one")
        old = self.commit("two")
        self.push(old, MAIN)
        self.git(self.dev, "reset", "-q", "--hard", start)
        new = self.commit("rewrite", GIT_AUTHOR_EMAIL="zq4x@example.invalid")
        self.push(new, MAIN, force=True)
        runner = self.runner_at(new)
        with self.assertRaises(subprocess.CalledProcessError):
            self.git(runner, "cat-file", "-e", old + "^{commit}")
        code, out, base, head = self.run_script(runner, "push", old)
        self.assertEqual((code, base, head), (0, old, new), out)
        patterns = os.path.join(self.work, "patterns.json")
        with open(patterns, "w", encoding="utf-8") as fh:
            json.dump({"patterns": [{"id": "tok", "regex": r"\bZQ4X\b", "flags": "i"}]}, fh)
        self.assertEqual(author_scan.captured_scan(patterns, runner, base, head)[0], author_scan.EXIT_FINDINGS)

    def test_force_push_to_another_branch_with_the_old_tip_gone_reads_over_main(self):
        start = self.commit("one")
        self.push(start, MAIN)
        old = self.commit("feature v1")
        self.push(old, "refs/heads/feature")
        self.git(self.dev, "reset", "-q", "--hard", start)
        new = self.commit("feature v2")
        self.push(new, "refs/heads/feature", force=True)
        self.drop_unreferenced()
        code, out, base, head = self.run_script(self.runner_at(new), "push", old, "refs/heads/feature")
        self.assertEqual((code, base, head), (0, start, new), out)

    def test_new_branch_reads_over_main(self):
        start = self.commit("one")
        self.push(start, MAIN)
        self.commit("feature a")
        new = self.commit("feature b")
        self.push(new, "refs/heads/feature")
        code, out, base, head = self.run_script(self.runner_at(new), "push", ZERO, "refs/heads/feature")
        self.assertEqual((code, base, head), (0, start, new), out)
        self.assertIn("2 commit(s).", out)

    def test_first_push_of_main_reads_every_commit(self):
        self.commit("one")
        new = self.commit("two")
        self.push(new, MAIN)
        code, out, base, head = self.run_script(self.runner_at(new), "push", ZERO, MAIN)
        self.assertEqual((code, base, head), (0, "", new), out)
        self.assertIn("2 commit(s).", out)

    def test_new_branch_with_no_main_reads_every_commit(self):
        new = self.commit("one")
        self.push(new, "refs/heads/feature")
        code, out, base, head = self.run_script(self.runner_at(new), "push", ZERO, "refs/heads/feature")
        self.assertEqual((code, base, head), (0, "", new), out)

    def test_pull_request_reads_the_pull_request_side_of_the_trial_merge(self):
        start = self.commit("one")
        self.git(self.dev, "checkout", "-q", "-b", "pr")
        pr_head = self.commit("pr work")
        self.git(self.dev, "checkout", "-q", "main")
        main_tip = self.commit("main moved on")
        self.git(self.dev, "merge", "-q", "--no-ff", "-m", "trial merge", "pr")
        trial = self.git(self.dev, "rev-parse", "HEAD")
        self.push(main_tip, MAIN)
        self.push(trial, "refs/pull/1/merge")
        code, out, base, head = self.run_script(self.runner_at(trial), "pull_request", "", "refs/pull/1/merge")
        self.assertEqual((code, base, head), (0, main_tip, pr_head), out)
        self.assertNotEqual(start, base)

    def test_previous_tip_that_is_not_a_commit_id_is_a_fault(self):
        self.commit("one")
        new = self.commit("two")
        self.push(new, MAIN)
        code, out, _, _ = self.run_script(self.runner_at(new), "push", "HEAD~1", MAIN)
        self.assertEqual(code, 3, out)

    def test_unsupported_event_is_a_fault(self):
        new = self.commit("one")
        self.push(new, MAIN)
        code, out, _, _ = self.run_script(self.runner_at(new), "workflow_dispatch")
        self.assertEqual(code, 3, out)
        self.assertIn("unsupported event", out)


if __name__ == "__main__":
    unittest.main()
