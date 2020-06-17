#!/bin/bash

FeedAndCheckEnv() {
    RISK=$1
    DOCKER_REPO=$2
    TARGET_PLATFORMS=$3

    if ! command -v docker >/dev/null 2>&1; then
        echo "Command 'docker' is missing"
        exit 1
    fi

    if ! command -v jq >/dev/null 2>&1; then
        echo "Command 'jq' is missing"
        exit 1
    fi

    if [ -z "${RISK}" ]; then
        echo "RISK is not specified"
        exit 1
    fi

    if [ -z "${DOCKER_REPO}" ]; then
        echo "DOCKER_REPO is not specified"
        exit 1
    fi

    if [ -z "${TARGET_PLATFORMS}" ]; then
        echo "TARGET_PLATFORMS is not specified"
        exit 1
    fi
}

ResolveArch() {
    PLATFORM=$1

    case "${PLATFORM}" in
        linux/amd64)
            SNAP_ARCH="amd64"
            QEMU_ARCH="x86_64"
            DOCKER_ARCH="amd64"
            MANIFEST_ARCH="amd64"
            ;;
        linux/386)
            SNAP_ARCH="i386"
            QEMU_ARCH="i386"
            DOCKER_ARCH="i386"
            MANIFEST_ARCH="386"
            ;;
        linux/arm64)
            SNAP_ARCH="arm64"
            QEMU_ARCH="aarch64"
            DOCKER_ARCH="arm64v8"
            MANIFEST_ARCH="arm64"
            ;;
        linux/arm/v7)
            SNAP_ARCH="armhf"
            QEMU_ARCH="arm"
            DOCKER_ARCH="arm32v7"
            MANIFEST_ARCH="arm/v7"
            ;;
        *)
            echo "Not supported build architecture '$1'." >&2
            exit 1
    esac
}

DownloadQemuStatic() {
    QEMU_ARCH=$1

    if [ ! -f "${DIR}/qemu-${QEMU_ARCH}-static" ]; then
        QEMU_DOWNLOAD_URL="https://github.com/multiarch/qemu-user-static/releases/download"
        QEMU_LATEST_TAG=$(curl -s https://api.github.com/repos/multiarch/qemu-user-static/tags | jq -r '.[0].name')
        echo "${QEMU_DOWNLOAD_URL}/${QEMU_LATEST_TAG}/x86_64_qemu-${QEMU_ARCH}-static.tar.gz"
        curl -SL "${QEMU_DOWNLOAD_URL}/${QEMU_LATEST_TAG}/x86_64_qemu-${QEMU_ARCH}-static.tar.gz" \
            | tar xzv -C "${DIR}"
    fi
}

BuildImage() {
    DOCKER_REPO=$1
    RISK=$2
    PLATFORM=$3

    ResolveArch "${PLATFORM}"

    DOCKER_IMAGE="${DOCKER_REPO}:${DOCKER_ARCH}-${RISK}"

    DownloadQemuStatic "${QEMU_ARCH}"

    docker build \
        --build-arg "SNAP_ARCH=${SNAP_ARCH}" \
        --build-arg "QEMU_ARCH=${QEMU_ARCH}" \
        --build-arg "DOCKER_ARCH=${DOCKER_ARCH}" \
        --build-arg "RISK=${RISK}" \
        --tag "${DOCKER_IMAGE}" \
        "${DIR}"
    FixManifest "${DOCKER_IMAGE}" "${MANIFEST_ARCH}"
}

FixManifest() {
    DOCKER_IMAGE=$1
    MANIFEST_ARCH=$2

    TEMPDIR=$(mktemp -d)
    pushd "${TEMPDIR}" || return 1

    docker save "${DOCKER_IMAGE}" | tar xv

    for FILENAME in */json; do
        [ -e "${FILENAME}" ] || continue
        jq --compact-output 'del(.architecture)' < "${FILENAME}" > "${FILENAME}.edited"
        mv -f "${FILENAME}.edited" "${FILENAME}"
    done

    for FILENAME in *.json; do
        [ -e "${FILENAME}" ] || continue
        ! [ "${FILENAME}" = "manifest.json" ] || continue
        jq --arg architecture "${MANIFEST_ARCH}" --compact-output '.architecture=$architecture' < "${FILENAME}" > "${FILENAME}.edited"
        mv -f "${FILENAME}.edited" "${FILENAME}"
    done

    tar cv . | docker load
    popd || return 1
    rm -rf "${TEMPDIR}"
}
