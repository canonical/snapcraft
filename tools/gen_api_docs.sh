#!/bin/bash

set -ev

sphinx-apidoc -o tools/api snapcraft snapcraft/tests/*
make -C docs/api html
