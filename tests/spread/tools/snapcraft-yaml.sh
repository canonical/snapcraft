#!/bin/bash -e

get_base()
{
    if [[ "$SPREAD_SYSTEM" =~ ubuntu-24.04 ]]; then
        echo "core24"
    elif [[ "$SPREAD_SYSTEM" =~ ubuntu-22.04 ]]; then
        echo "core22"
    elif [[ "$SPREAD_SYSTEM" =~ ubuntu-20.04 ]]; then
        echo "core20"
    else
        exit 1
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
        sed -i "s/^base:.*/base: $base/g" "$snapcraft_yaml_path"
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

restore_yaml()
{
    apt-get install --yes git
    apt-mark auto git

    snapcraft_yaml_path="$1"

    # Restoration only really works for git committed snapcraft.yaml's
    git checkout "$snapcraft_yaml_path"
}

get_stage_dir()
{
    if [[ "$SPREAD_SYSTEM" =~ ubuntu-24.04 ]]; then
        echo "stage/default"
    elif [[ "$SPREAD_SYSTEM" =~ ubuntu-22.04 ]]; then
        echo "stage/default"
    elif [[ "$SPREAD_SYSTEM" =~ ubuntu-20.04 ]]; then
        echo "stage"
    else
        exit 1
    fi
}

get_prime_dir()
{
    if [[ "$SPREAD_SYSTEM" =~ ubuntu-24.04 ]]; then
        echo "prime/default"
    elif [[ "$SPREAD_SYSTEM" =~ ubuntu-22.04 ]]; then
        echo "prime/default"
    elif [[ "$SPREAD_SYSTEM" =~ ubuntu-20.04 ]]; then
        echo "prime"
    else
        exit 1
    fi
}
