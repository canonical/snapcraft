# Creating docker containers for snapcraft

By default the `Dockerfile` builds Ubuntu 16.04 (Xenial) image with `snapcraft` from the `edge` channel.

    docker build . --no-cache

It is however possible to choose the base Ubuntu version and the Snapcraft channel (risk levels):

- `edge`
- `beta`
- `candidate`
- `stable`

To do that, use `--build-arg RISK=<risk>` and `--build-arg UBUNTU=<name>` arguments:

    docker build . --no-cache --build-arg RISK=beta --build-arg UBUNTU=bionic
