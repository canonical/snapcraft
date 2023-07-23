# Creating docker containers for snapcraft

By default the `Dockerfile` builds Ubuntu 22.04 (Jellyfish) image with
`snapcraft` from the `stable` channel:

    docker build . --no-cache

To choose different Ubuntu version and the Snapcraft channel (risk levels),
use use `--build-arg RISK=<risk>` and `--build-arg UBUNTU=<name>` options:

    docker build . --no-cache --build-arg RISK=beta --build-arg UBUNTU=bionic

Possible RISK values:

- `edge`
- `beta`
- `candidate`
- `stable`

