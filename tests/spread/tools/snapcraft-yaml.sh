#!/bin/bash -e

set_base()
{
    snapcraft_yaml_path="$1"

    if [[ "$SPREAD_SYSTEM" =~ ubuntu-18.04 ]]; then
        base="core18"
    elif [[ "$SPREAD_SYSTEM" =~ ubuntu-16.04 ]]; then
        base="core16"
    else
        echo "Test not supported for $SPREAD_SYSTEM"
        exit 1
    fi
    
    # Insert at the very top to be safe
    sed -i "1ibase: $base"  "$snapcraft_yaml_path"
}

restore_yaml()
{
    snapcraft_yaml_path="$1"

    # Restoration only really works for git committed snapcraft.yaml's
    git checkout "$snapcraft_yaml_path"
}
