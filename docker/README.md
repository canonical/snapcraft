# Creating Docker images for snapcraft

## Setup

    sudo snap install docker jq

Ensure networking works, in some cases `dockerd` might need to have something
like `--dns=8.8.8.8` added to the command.

## Creating images

You can create the images using the following command:
```
docker/build.sh RISK DOCKER_REPO PLATFORMS
```

Here are the meaning and possible values for each parameter:
* `RISK` is the risk level of snapcraft, can be:
    + `stable`
    + `candidate`
    + `beta`
    + `edge`
* `DOCKER_REPO` is the base image name (eg. `snapcore/snapcraft`)
* `PLATFORMS` is a comma-separated list of target for which the Docker is built, can be:
    + `linux/amd64` (arch `amd64`)
    + `linux/386` (arch `i386`)
    + `linux/arm64` (arch `arm64v8`)
    + `linux/arm/v7` (arch `arm32v7`)

Image(s) created will be labeled as `DOCKER_REPO:ARCH-RISK`.

For instance, to build images of `snapcore/snapcraft` for `linux/amd64` and `linux/arm64`
with `stable` risk level, run:
```
docker/build.sh stable snapcore/snapcraft linux/amd64,linux/arm64
```

You will get the following images:
* `snapcore/snapcraft:amd64-stable`
* `snapcore/snapcraft:arm64v8-stable`

## Pushing images and multi-arch manifest

You can push the images using the following command:
```
docker/publish.sh RISK DOCKER_REPO PLATFORMS
```

`RISK`, `DOCKER_REPO` and `PLATFORMS` have the same meaning and possible values than in
the [Creating images](#creating-images) section.

On top of publishing each architecture specific `DOCKER_REPO:ARCH-RISK` image, this script
will also publish a multi-arch manifest named `DOCKER_REPO:RISK` that can be pulled by users:
the local Docker daemon will automatically download the architecture specific image
corresponding to the host architecture.
