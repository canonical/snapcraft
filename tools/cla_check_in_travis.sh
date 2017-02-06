#!/bin/bash

set -ev

echo $TRAVIS_COMMIT_RANGE
error='false'
for email in $(git log $TRAVIS_COMMIT_RANGE --pretty="%aE"); do
    echo "Checking the licence agreement for $email..."

    if [ "$(echo $email | cut -d @ -f 2)" = 'canonical.com' ]; then
        echo 'Branch from a Canonical employee. CLA OK.'
    elif ./tools/cla_check_in_launchpad.py $email; then
        echo 'Branch from a signed contributor. CLA OK.'
    else
        echo 'Cannot confirm that the contributor has signed the agreement.'
        error='true'
    fi
done

if [ "$error" == 'true' ]; then exit 1; fi
