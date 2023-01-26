#!/bin/bash -e

install_snapcraft()
{
  # If $SNAPCRAFT_CHANNEL is defined, install snapcraft from that channel.
  # Otherwise, look for it in /snapcraft/.
  if [ -z "$SNAPCRAFT_CHANNEL" ]; then
    if stat /snapcraft/tests/*.snap 2>/dev/null; then
      snap install --classic --dangerous /snapcraft/tests/*.snap
    else
      echo "Expected a snap to exist in /snapcraft/tests/. If your intention"\
           "was to install from the store, set \$SNAPCRAFT_CHANNEL."
      exit 1
    fi
  else
    snap install --classic snapcraft --channel="$SNAPCRAFT_CHANNEL"
  fi
}
