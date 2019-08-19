# Creating docker containers for snapcraft

## Setup

    sudo snap install docker

Ensure networking works, in some cases `dockerd` might need to have something
like `--dns=8.8.8.8` added to the command.

## Creating containers

The Dockerfile here can build images for these risk levels:

- **edge**: Using the snap from edge
- **beta**: Using the snap from beta
- **candidate**: Using the snap from candidate
- **stable**: Using the snap from stable

By default, the `edge` image will be built. Pass `--build-arg RISK=<risk>` to
choose needed risk level for target container:

    export RISK=beta
    docker build --no-cache --tag snapcore/snapcraft:$RISK --build-arg RISK=$RISK .

You can push that image with:

    docker push snapcore/snapcraft:$RISK
