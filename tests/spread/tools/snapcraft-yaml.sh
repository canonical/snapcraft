#!/bin/bash -e

get_base()
{
    if [[ "$SPREAD_SYSTEM" =~ ubuntu-20.04 ]]; then
        echo "core20"
    elif [[ "$SPREAD_SYSTEM" =~ ubuntu-18.04 ]]; then
        echo "core18"
    elif [[ "$SPREAD_SYSTEM" =~ ubuntu-16.04 ]]; then
        echo "core"
    else
	echo ""
    fi
}

set_base()
{
    snapcraft_yaml_path="$1"

    base="$(get_base)"
    if [[ -z "$base" ]]; then
        echo "Test not supported for $SPREAD_SYSTEM"
        exit 1
    fi

    if grep -q "^base:" "$snapcraft_yaml_path"; then
        sed -i "s/base:.*/base: $base/g" "$snapcraft_yaml_path"
    else
        # Insert at the very top to be safe
        sed -i "1ibase: $base"  "$snapcraft_yaml_path"
    fi
}

clear_base()
{
    snapcraft_yaml_path="$1"
    sed -i '/^base:/d' "$snapcraft_yaml_path"
}

set_name()
{
    snapcraft_yaml_path="$1"
    name="$2"

    sed -i -e "s/name: .*$/name: $name/" "$snapcraft_yaml_path"
}

set_confinement()
{
    snapcraft_yaml_path="$1"
    confinement="$2"

    if grep -q "^confinement: " "$snapcraft_yaml_path"; then
        sed -i -e "s/confinement: \w*$/confinement: $confinement/" "$snapcraft_yaml_path"
    else
        sed -i "1iconfinement: $confinement"  "$snapcraft_yaml_path"
    fi
}

set_grade()
{
    snapcraft_yaml_path="$1"
    grade="$2"

    if grep -q "^grade: " "$snapcraft_yaml_path"; then
        sed -i -e "s/grade: \w*$/grade: $grade/" "$snapcraft_yaml_path"
    else
        sed -i "1igrade: $grade"  "$snapcraft_yaml_path"
    fi
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
    apt-get install --yes git
    apt-mark auto git

    snapcraft_yaml_path="$1"

    # Restoration only really works for git committed snapcraft.yaml's
    git checkout "$snapcraft_yaml_path"
}
