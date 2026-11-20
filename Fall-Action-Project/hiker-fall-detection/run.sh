#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
stage="4"
if (( $# > 0 )) && [[ "$1" != -* ]]; then
  stage="$1"
  shift
fi
case "$stage" in
  1|stock-single) exec "$ROOT/iterations/01_stock_single/run.sh" "$@" ;;
  2|custom-single) exec "$ROOT/iterations/02_custom_single/run.sh" "$@" ;;
  3|custom-four) exec "$ROOT/iterations/03_custom_four_people/run.sh" "$@" ;;
  4|custom-eight|latest) exec "$ROOT/iterations/04_custom_eight_full_pose/run.sh" "$@" ;;
  test) exec "$ROOT/test.sh" ;;
  help|-h|--help)
    echo "Usage: ./run.sh {1|2|3|4|test} [detector options]"
    echo "Default: stage 4. Add --source 0 --lying-seconds 10, or --headless."
    ;;
  *) echo "Unknown stage: $stage" >&2; exit 2 ;;
esac

