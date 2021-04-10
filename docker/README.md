# Creating docker containers for snapcraft

By default, the `Dockerfile` builds Ubuntu 16 (xenial) image with `snapcraft` from the `edge` channel.

    docker build . --no-cache

It is however possible to choose base Ubuntu version and one of these channels (risk levels):

- `edge`
- `beta`
- `candidate`
- `table`

To do that, use `--build-arg RISK=<risk>` and `--build-arg UBUNTU=<name>` arguments:

    docker build . --no-cache --build-arg RISK=beta --build-arg UBUNTU=bionic
