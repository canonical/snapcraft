# Creating docker containers for snapcraft

## Setup

    sudo snap install docker

Ensure networking works, in some cases `dockerd` might need to have something
like `--dns=8.8.8.8` added to the command.

## Creating containers

Note that in the following commands, `--network host` is not strictly needed
if a proper docker bridge is setup.

### snapcraft snap on edge

    docker build --no-cache -f snap-edge.Dockerfile --label snapcore/snapcraft --tag snapcraft:edge --network host .
    docker push snapcore/snapcraft:edge

### snapcraft snap on beta

    docker build --no-cache -f snap-beta.Dockerfile --label snapcore/snapcraft --tag snapcraft:beta --network host .
    docker push snapcore/snapcraft:beta
