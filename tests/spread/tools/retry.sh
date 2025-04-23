#!/bin/bash -e

retry() {
  # retry a failing command 3 times, with a 1 minute delay between attempts
  local max_attempts=3
  local delay=60
  local attempt=1

  while true; do
    # shellcheck disable=SC2015
    "$@" && break || {
      if (( attempt == max_attempts )); then
        echo "Command failed after $max_attempts attempts." >&2
        return 1
      else
        echo "Command failed (attempt $attempt/$max_attempts) Retrying in $delay seconds..." >&2
        sleep $delay
        ((attempt++))
      fi
    }
  done
}
