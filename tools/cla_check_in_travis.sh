#!/bin/bash

set -ev

email="$(git log -1 $TRAVIS_PULL_REQUEST_SHA --pretty="%aE")"
echo "Checking the licence agreement for $email..."

if [ "$(echo $email | cut -d @ -f 2)" = 'canonical.com' ]; then
    echo 'Branch from a Canonical employee. CLA OK.'
    exit 0
elif ./tools/cla_check_in_launchpad.py $email; then
    echo 'Branch from a signed contributor. CLA OK.'
    exit 0
else
    echo 'Cannot confirm that the contributor has signed the agreement.'
    exit 1
fi
