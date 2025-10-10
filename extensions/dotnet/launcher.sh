#!/usr/bin/env bash

if [ -z "${SNAP}" ]; then
  echo "Not running inside a snap context: SNAP not declared"
  exit 1
fi

if ! snapctl is-connected "${DOTNET_EXT_PLUG_NAME}"; then
  >&2 echo "Plug '${DOTNET_EXT_PLUG_NAME}' isn't connected."
  >&2 echo "Please run: 'snap connect ${DOTNET_EXT_SNAP_NAME}:${DOTNET_EXT_PLUG_NAME} ${DOTNET_EXT_CONTENT_SNAP}:${DOTNET_EXT_PLUG_NAME}'."
  exit 1
fi

if [[ -f ${DOTNET_ROOT}/dotnet ]]; then
  exec "$@"
else
  echo "No .NET Runtime found."
  echo "DOTNET_ROOT is set to '${DOTNET_ROOT}'"
  exit 1
fi
