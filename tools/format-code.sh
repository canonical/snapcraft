#!/bin/bash -x

# Sort imports first (order matters at the moment due to a minor
# difference between isort & black. See:
# https://github.com/psf/black/issues/251
isort -y

# Apply formatting.
black .
