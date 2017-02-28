#!/bin/bash

set -ev

GH_REPO="@github.com/snapcore/snapcraft.git"

FULL_REPO="https://$GH_TOKEN$GH_REPO"

cd docs/api/_build/html

git init
git config user.name "snappy-m-o"
git config user.email "travis"

# Do not use Jeykyll
touch .nojekyll

git add .
git commit -m "Deployed to github pages."
git push --force --quiet $FULL_REPO master:gh-pages
