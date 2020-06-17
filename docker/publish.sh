#!/bin/bash
# Publish the Docker images of snapcraft for the given risk and target platforms + a multi-arch manifest.
# Usage: publish.sh RISK DOCKER_REPO TARGET_PLATFORMS
#  - RISK: risk value for snapcraft source, can be edgem, beta, candidate or stable
#  - DOCKER_REPO: Docker image to build
#  - TARGET_PLATFORMS: comma-separated list of target platforms, possible values are
#    linux/amd64, linux/arm64, linux/386 or linux/arm/v7
set -ex

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# shellcheck source=docker/common.sh
. "${DIR}/common.sh"

FeedAndCheckEnv "$@"

MANIFEST_LIST=()

IFS=$','
for TARGET_PLATFORM in ${TARGET_PLATFORMS}; do
    ResolveArch "${TARGET_PLATFORM}"

    DOCKER_IMAGE="${DOCKER_REPO}:${DOCKER_ARCH}-${RISK}"

    docker push "${DOCKER_IMAGE}"
    MANIFEST_LIST+=("${DOCKER_IMAGE}")
done

export DOCKER_CLI_EXPERIMENTAL=enabled
# shellcheck disable=SC2086
docker manifest create --amend "${DOCKER_REPO}:${RISK}" ${MANIFEST_LIST[*]}
docker manifest push --purge "${DOCKER_REPO}:${RISK}"
