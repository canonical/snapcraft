# Snapcraft

## Setting up a development environment

We want to make sure everyone develops using a consistent base, to ensure that these instructions rely on LXD (use whatever is convenient as long as you do not stray away from an Ubuntu 16.04 LTS base)

First we setup LXD:

    sudo snap install lxd
    sudo lxd init  # If unsure, pick `dir` as the storage backend.
    sudo adduser "$USER" lxd
    newgrp lxd
    lxc create ubuntu:16.04 snapcraft-dev
    lxc config set snapcraft-dev raw.idmap "both $UID 1000"
    lxc config device add snapcraft-dev snapcraft-project disk source="$(PWD)" path=/home/ubuntu/snapcraft
    lxc start snapcraft-dev

The we install the required dependencies:

    lxc exec snapcraft-dev -- apt update
    lxc exec snapcraft-dev -- apt install \
        execstack \
        g++ \
        gcc \
        libapt-pkg-dev \
        libffi-dev \
        libsodium-dev \
        libxml2-dev \
        libxslt-dev
        libyaml-dev \
        make \
        p7zip-full \
        patchelf \
        python3-dev \
        python3-venv \
        rpm2cpio \
        squashfs-tools \
    lxc exec snapcraft-dev -- sudo -u ubuntu pip3 install -U pip
    lxc exec snapcraft-dev -- sudo -u ubuntu pip install \
        -r snapcraft/requirements.txt \
        -r snapcraft/requirements-devel.txt

Optionally, to quickly try out snapcraft from within the environment:

    lxc exec snapcraft-dev -- sudo -u ubuntu pip install --editable snapcraft

> Import your keys (`ssh-import-id`) and add a `Host` entry to your ssh config if you are interested in [Code's](https://snapcraft.io/code) [Remote-SSH]() plugin.

### Testing

See the [Testing guide](TESTING.md).

### Enabling debug output

Given that the `--debug` option in snapcraft is reserved for project specific debugging, enabling for the `logger.debug` calls is achieved by setting the "SNAPCRAFT_ENABLE_DEVELOPER_DEBUG" environment variable to a truthful value. Snapcraft's internal tools, e.g.; `snapraftctl` should pick up this environment variable as well.

## Evaluating pull requests

Oftentimes all you want to do is see if a given pull request solves the issue you were having. To make this easier, the Travis CI setup for snapcraft _publishes_ the resulting snap that was built for x86-64 using `transfer.sh`.
To download the snap, find the relevant CI job run for the PR under review and locate the "snap" stage, the URL to download from will be located at the end of logs for that job.

## Reaching out

We'd love the help!

- Submit pull requests against [snapcraft](https://github.com/snapcore/snapcraft/pulls)
- Make sure to read the [contribution guide](CONTRIBUTING.md)
- Find us under the snapcraft category of the forum https://forum.snapcraft.io
- Discuss with us using IRC in #snapcraft on Freenode.
