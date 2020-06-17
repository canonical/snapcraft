#!/bin/bash
# Build Docker images of snapcraft for the given risk and target platforms.
# Usage: build.sh RISK DOCKER_REPO TARGET_PLATFORMS
#  - RISK: risk value for snapcraft source, can be edgem, beta, candidate or stable
#  - DOCKER_REPO: Docker image to build
#  - TARGET_PLATFORMS: comma-separated list of target platforms, possible values are
#    linux/amd64, linux/arm64, linux/386 or linux/arm/v7
set -ex

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# shellcheck source=docker/common.sh
. "${DIR}/common.sh"

FeedAndCheckEnv "$@"

docker run --rm --privileged multiarch/qemu-user-static:register --reset

IFS=$','
for TARGET_PLATFORM in ${TARGET_PLATFORMS}; do
    BuildImage "${DOCKER_REPO}" "${RISK}" "${TARGET_PLATFORM}"
done
