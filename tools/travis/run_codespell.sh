#!/bin/sh
set -e
sudo pip install codespell
codespell -S "*.xz,*.zip,*.bz2,*.7z,*.gz,*.deb,*.rpm,*.snap,*.gpg,*.pyc,*.png,*.ico,*.jar,./.git" -q4
