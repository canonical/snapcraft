#!/bin/bash -e

# Run review-tools on a snap and print the results as JSON.
#
# The review-tools snap can only read files in $HOME, so the snap is copied
# to a temporary directory there and removed after the review.
#
# The review fails on any error or warning. If a snap is expected to have
# findings, use --no-fail and check only the "error" and "warn" lists, as the
# output also includes passing checks. To check that a list is empty, count it.
# Save each yq result before matching it, so a yq error still fails the test:
#
#   review --no-fail classic-patchelf_0.1_amd64.snap > review.json
#   report=$(cat review.json)
#   errors=$(printf '%s\n' "$report" | yq -p json -oy '[.[].error | keys | .[]] | join(",")' -)
#   warnings=$(printf '%s\n' "$report" | yq -p json -oy '[.[].warn | keys | .[]] | length' -)
#   echo "$errors" | MATCH '^lint-snap-v2:confinement_classic$'
#   echo "$warnings" | MATCH '^0$'
review() (
  set +e
  local no_fail=0
  local staging=""
  local status=0
  local artifact=""
  local output=""

  # Remove the copy on exit, keeping the review's exit status.
  trap '
    status=$?
    if [[ -n "$staging" ]]; then
      rm -rf -- "$staging"
    fi
    exit "$status"
  ' EXIT

  if [[ "${1:-}" == "--no-fail" ]]; then
    no_fail=1
    shift
  fi

  if [[ $# -ne 1 || "${1:-}" == -* ]]; then
    echo "review: usage: review [--no-fail] <snap>" >&2
    return 1
  fi

  artifact="$1"
  if [[ ! -f "$artifact" || ! -r "$artifact" ]]; then
    echo "review: snap not found or not readable: $artifact" >&2
    return 1
  fi
  case "$artifact" in
  *.snap) ;;
  *)
    echo "review: not a .snap file: $artifact" >&2
    return 1
    ;;
  esac

  if ! command -v review-tools.snap-review >/dev/null 2>&1; then
    echo "review: review-tools.snap-review not found" >&2
    return 1
  fi

  staging="$(mktemp -d "$HOME/snap-review-XXXXXX")" || {
    echo "review: could not create a staging directory under \$HOME" >&2
    return 1
  }

  if ! cp -- "$artifact" "$staging/"; then
    echo "review: could not copy snap into $staging" >&2
    return 1
  fi

  # Confined review-tools can't write to a redirected file, so capture it.
  output="$(review-tools.snap-review --json -- "$staging/${artifact##*/}")"
  status=$?
  printf '%s\n' "$output"

  if [[ "$no_fail" -eq 1 && ("$status" -eq 2 || "$status" -eq 3) ]]; then
    return 0
  fi
  return "$status"
)
