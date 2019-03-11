#!/bin/bash -e

set_base()
{
    snapcraft_yaml_path="$1"

    if [[ "$SPREAD_SYSTEM" =~ ubuntu-18.04 ]]; then
        base="core18"
    elif [[ "$SPREAD_SYSTEM" =~ ubuntu-16.04 ]]; then
        # Use core instead of core16 (LP: #1819290)
        base="core"
    else
        echo "Test not supported for $SPREAD_SYSTEM"
        exit 1
    fi
    
    # Insert at the very top to be safe
    sed -i "1ibase: $base"  "$snapcraft_yaml_path"
}

set_openjdk_version()
{
    snapcraft_yaml_path="$1"
    plugin="$2"
    openjdk_version="$3"

    # if openjdk_version is not set to LATEST, then there is nothing
    # to be done
    if [[ "$openjdk_version" != "LATEST" ]]; then
        return
    fi

    if [[ "$SPREAD_SYSTEM" =~ ubuntu-18.04 ]]; then
        openjdk_version="11"
    elif [[ "$SPREAD_SYSTEM" =~ ubuntu-16.04 ]]; then
        openjdk_version="9"
    else
        echo "Test not supported for $SPREAD_SYSTEM"
        exit 1
    fi

    sed -i -e "s/\(\s$plugin-openjdk-version:\).*/\1 \"$openjdk_version\"/" "$snapcraft_yaml_path"
}

restore_yaml()
{
    snapcraft_yaml_path="$1"

    # Restoration only really works for git committed snapcraft.yaml's
    git checkout "$snapcraft_yaml_path"
}
