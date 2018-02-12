# Creating docker containers for snapcraft

## Setup

    sudo snap install docker

Ensure networking works, in some cases `dockerd` might need to have something
like `--dns=8.8.8.8` added to the command.

## Creating containers

There are four separate Dockerfiles here, each one corresponding to their
respective risk level:

- **edge.Dockerfile**: Using the snap from edge
- **beta.Dockerfile**: Using the snap from beta
- **candidate.Dockerfile**: Using the snap from candidate
- **stable.Dockerfile**: Using the snap from stable

Build the docker image with the following (note that `--network host` is not
strictly needed if a proper docker bridge is setup):

    docker build --no-cache -f <risk>.Dockerfile --label snapcore/snapcraft --tag snapcore/snapcraft:<risk> --network host .

You can push that image with:

    docker push snapcore/snapcraft:<risk>
