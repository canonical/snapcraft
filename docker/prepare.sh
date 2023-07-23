#!/bin/bash

set -xeo pipefail


echo "--- unpacking bases for snapcraft/$RISK ---"

# Grab the core snaps (which snapcraft uses as a base) from the stable channel
# and unpack them in the proper place.
BASES="core core18 core20 core22"
for base in $BASES; do
    curl -L "$(curl -H "X-Ubuntu-Series: 16" "https://api.snapcraft.io/api/v1/snaps/details/$base" | jq ".download_url" -r)" --output "$base.snap"
    mkdir -p "/snap/$base"
    unsquashfs -d "/snap/$base/current" "$base.snap"
done


echo "--- unpacking snapcraft/$RISK ---"

# Grab the snapcraft snap from the $RISK channel and unpack it in the proper
# place.
curl -L "$(curl -H "X-Ubuntu-Series: 16" "https://api.snapcraft.io/api/v1/snaps/details/snapcraft?channel=$RISK" | jq ".download_url" -r)" --output snapcraft.snap
mkdir -p /snap/snapcraft
unsquashfs -d /snap/snapcraft/current snapcraft.snap
