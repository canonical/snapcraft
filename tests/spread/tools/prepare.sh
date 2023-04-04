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

install_snapcraft_as()
{
  # install snapcraft with a particular name
  # If $SNAPCRAFT_CHANNEL is defined, install snapcraft from that channel.
  # Otherwise, look for it in /snapcraft/.
  if [ -z "$SNAPCRAFT_CHANNEL" ]; then
    if stat /snapcraft/tests/*.snap 2>/dev/null; then
      snap install --classic --dangerous /snapcraft/tests/*.snap --name "$1"
    else
      echo "Expected a snap to exist in /snapcraft/tests/. If your intention"\
           "was to install from the store, set \$SNAPCRAFT_CHANNEL."
      exit 1
    fi
  else
    snap install --classic snapcraft --channel="$SNAPCRAFT_CHANNEL" --name "$1"
  fi
}

install_lxd()
{
  # install and setup the lxd snap - use 'retry' to workaround aa-exec issue
  # see https://bugs.launchpad.net/snapd/+bug/1870201
  retry -n 5 --wait 5 sh -c 'snap install lxd'
  lxd init --auto
}
