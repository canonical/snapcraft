#!/bin/bash

set -ev

sphinx-apidoc -o docs/api snapcraft
make -C docs/api html
