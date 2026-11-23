#!/usr/bin/env bash
# Choose the commits a push or pull request adds, for the leak-check jobs.
#
#   EVENT=<github.event_name> BEFORE=<github.event.before> REF=<github.ref> push_range.sh
#
# Run from the checkout. Writes base=<sha, or empty for "every commit"> and head=<sha> to
# $GITHUB_OUTPUT (stdout when that is unset) and a one-line summary to stdout.
#
#   pull_request  The checkout is GitHub's trial merge, which never lands: base is its first
#                 parent (the base branch), head its second (the pull request head).
#   push          base is the previous tip, BEFORE. actions/checkout fetches branch tips and tags
#                 only, so after a force push the old tip is often absent; it is then fetched by
#                 its id. If it still cannot be read:
#                   on main  instrument fault. Falling back to the merge base with main would give
#                            base == head and scan nothing, which is how a rewritten main would
#                            have passed green.
#                   elsewhere  everything the branch adds over main.
#   new branch    (BEFORE all zeros) everything the branch adds over main; on main itself, or with
#                 no main to compare with, every commit reachable from head.
#
# Exit 0 with a range, 3 instrument fault (never a statement about the commits).
set -euo pipefail

ZERO=0000000000000000000000000000000000000000
MAIN_REF=refs/heads/main
EXIT_FAULT=3

fault() {
  echo "::error::INSTRUMENT FAULT (not a finding): $*"
  exit "$EXIT_FAULT"
}

have_commit() {
  git cat-file -e "$1^{commit}" 2>/dev/null
}

# The functions below set BASE rather than echo it: a fault inside $(...) would lose its message.
BASE=""

over_main() {
  # Everything the branch adds over main, or every commit when there is no main.
  if git rev-parse --verify --quiet origin/main >/dev/null; then
    BASE="$(git merge-base origin/main HEAD)" || fault "no merge base between origin/main and HEAD"
  else
    BASE=""
  fi
}

push_base() {
  local before="$1"
  if ! [[ "$before" =~ ^[0-9a-f]{40}$ ]]; then
    fault "the push's previous tip is not a commit id"
  fi
  if ! have_commit "$before"; then
    # A failed fetch is not itself a fault: the old tip may be gone, and the checks below decide.
    git fetch --quiet --no-tags origin "$before" 2>/dev/null || echo "previous tip not fetchable" >&2
  fi
  if have_commit "$before"; then
    BASE="$before"
  elif [ "${REF:-}" = "$MAIN_REF" ]; then
    fault "the previous tip of main (${before:0:12}) cannot be read, so the commits this push adds are unknown"
  else
    over_main
  fi
}

main() {
  local head
  case "${EVENT:-}" in
    pull_request)
      BASE="$(git rev-parse --verify --quiet 'HEAD^1')" || fault "the trial merge has no first parent"
      head="$(git rev-parse --verify --quiet 'HEAD^2')" || fault "the trial merge has no second parent"
      ;;
    push)
      head="$(git rev-parse HEAD)"
      if [ -n "${BEFORE:-}" ] && [ "$BEFORE" != "$ZERO" ]; then
        push_base "$BEFORE"
      elif [ "${REF:-}" = "$MAIN_REF" ]; then
        BASE=""
      else
        over_main
      fi
      ;;
    *)
      fault "unsupported event '${EVENT:-}'"
      ;;
  esac
  echo "base ${BASE:-none}; head $head; $(git rev-list --count ${BASE:+$BASE..}$head) commit(s)."
  {
    echo "base=$BASE"
    echo "head=$head"
  } >> "${GITHUB_OUTPUT:-/dev/stdout}"
}

main
