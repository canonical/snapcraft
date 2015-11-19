#!/bin/sh

# This only exists because we can't use $SNAP_APP_DATA_PATH in the `start`
# keyword for a service in `services`
exec shout --home $SNAP_APP_DATA_PATH
