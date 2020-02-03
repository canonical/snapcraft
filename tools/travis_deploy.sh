#!/bin/sh

if [ -z "$TRAVIS_PULL_REQUEST" ]; then
    echo "'TRAVIS_PULL_REQUEST' is not set."
    exit 1
fi


if [ -z "$SNAP_TOKEN" ]; then
    echo '"SNAP_TOKEN" is not set.'
    exit 1
fi

# Login
echo "$SNAP_TOKEN" | /snap/bin/snapcraft login --with -
/snap/bin/snapcraft push \
                    --release "edge/pr-$TRAVIS_PULL_REQUEST" \
                    "snapcraft-pr$TRAVIS_PULL_REQUEST.snap"
