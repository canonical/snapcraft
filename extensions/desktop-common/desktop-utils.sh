#!/bin/bash

function prepend_dir() {
  local var="$1"
  local dir="$2"
  if [ -d "$dir" ]; then
    eval "export $var=\"\$dir\${$var:+:\$$var}\""
  fi
}

function append_dir() {
  local var="$1"
  local dir="$2"
  if [ -d "$dir" ]; then
    eval "export $var=\"\${$var:+\$$var:}\$dir\""
  fi
}

function can_open_file() {
  head -c0 "$1" &> /dev/null
}

function is_subpath() {
  dir="$(realpath "$1")"
  parent="$(realpath "$2")"
  [ "${dir##$parent/}" != "$dir" ] && return 0 || return 1
}

# TMPDIR is declared by the snap runtime
DESKTOP_PIDS_PATH="${TMPDIR:-/tmp}/SNAP_DESKTOP_PIDS"

if [ -f "$DESKTOP_PIDS_PATH" ]; then
  # shellcheck source=/dev/null
  source "$DESKTOP_PIDS_PATH"
else
  declare -A PIDS
fi

function async_exec() {
  "$@" &
  PIDS[$!]=$*
}

function wait_for_async_execs() {
  for pid in "${!PIDS[@]}"
  do
    wait "$pid" && continue || echo "ERROR: ${PIDS[$pid]} exited abnormally with status $?"
  done
  rm -f "$DESKTOP_PIDS_PATH"
}

function export_async_pids() {
  declare -p PIDS > "$DESKTOP_PIDS_PATH"
}
