#!/usr/bin/env bash
# Close the judge's console (tmux `judge`: rc-tui + judge_web). stop_ocs.sh / stop_rctest.sh call it for the console they opened.
# usage: scripts/stop_judge.sh [--owner NAME]     (with --owner: only if that launcher opened it)      -h | --help
set -u
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
WANT=""; [ "${1:-}" = "--owner" ] && WANT="${2:-}"
if tmux has-session -t judge 2>/dev/null; then
  OWNER="$(tmux show-environment -t judge JUDGE_OWNER 2>/dev/null | cut -d= -f2)"
  if [ -n "$WANT" ] && [ "$OWNER" != "$WANT" ]; then echo "judge console left running (opened by ${OWNER:-?}, not $WANT)"; exit 0; fi
  for w in tui web; do tmux send-keys -t "judge:$w" C-c 2>/dev/null || true; done
  sleep 0.5; tmux send-keys -t judge:tui q 2>/dev/null || true; sleep 0.3
  tmux kill-session -t judge 2>/dev/null || true
  echo "judge console closed"
else
  echo "no judge console running"
fi
