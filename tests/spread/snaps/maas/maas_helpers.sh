#!/bin/bash -e

if [[ "${SPREAD_SYSTEM}" =~ ubuntu-20.04 ]]; then
    MAAS_TAG="2.9"
elif [[ "${SPREAD_SYSTEM}" =~ ubuntu-18.04 ]]; then
    MAAS_TAG="2.8"
else
    echo "System not supported."
    exit 1
fi


prepare() {
    add-apt-repository --yes --update "ppa:maas/${MAAS_TAG}-next"

    git clone --recursive -b "${MAAS_TAG}" "${SNAP_REPO}" repo
}

restore() {
    add-apt-repository --yes --update --remove "ppa:maas/${MAAS_TAG}-next"

    if [ ! -d repo ]; then
        return
    fi

    pushd repo
    snapcraft clean "${PROVIDER_OPTION}"
    popd

    rm -rf repo
}

execute() {
    if [ ! -d repo ]; then
       echo "Need to setup first."
       exit 1
    fi

    pushd repo
    snapcraft "${PROVIDER_OPTION}"
    snap install --dangerous maas_*.snap
    popd

    output=$(maas init --help)
    echo "${output}" | head -1 | MATCH "usage: maas init"
    
}
