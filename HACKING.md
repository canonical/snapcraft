# Snapcraft

## Setting up a development environment

We want to make sure everyone develops using a consistent base, to ensure that these instructions rely on LXD (use whatever is convenient as long as you do not stray away from an Ubuntu LTS base)

Clone the snapcraft repository and its submodules and make it your working directory:

```shell
git clone https://github.com/snapcore/snapcraft.git --recurse-submodules
cd snapcraft
```

If you already have LXD setup you can skip this part, if not, run:

```shell
sudo snap install lxd
sudo lxd init --auto --storage-backend=dir
sudo adduser "$USER" lxd
newgrp lxd
```

Setup the environment by running:

```shell
./tools/environment-setup.sh
```

To work inside this environment, run:

```shell
lxc exec snapcraft-dev -- sudo -iu ubuntu bash
```

Import your keys (`ssh-import-id`) and add a `Host` entry to your ssh config if you are interested in [Code's](https://snapcraft.io/code) [Remote-SSH](https://code.visualstudio.com/docs/remote/ssh) plugin.

### Testing

See the [Testing guide](TESTING.md).

### Enabling debug output

Given that the `--debug` option in snapcraft is reserved for project specific debugging, enabling for the `logger.debug` calls is achieved by setting the "SNAPCRAFT_ENABLE_DEVELOPER_DEBUG" environment variable to a truthful value. Snapcraft's internal tools, e.g.; `snapcraftctl` should pick up this environment variable as well.

## Evaluating pull requests

Oftentimes all you want to do is see if a given pull request solves the issue you were having. To make this easier, the Travis CI setup for snapcraft _publishes_ the resulting snap that was built for x86-64 using `transfer.sh`.
To download the snap, find the relevant CI job run for the PR under review and locate the "snap" stage, the URL to download from will be located at the end of logs for that job.

## Reaching out

We'd love the help!

- Submit pull requests against [snapcraft](https://github.com/snapcore/snapcraft/pulls)
- Make sure to read the [contribution guide](CONTRIBUTING.md)
- Find us under the snapcraft category of the forum https://forum.snapcraft.io
- Discuss with us using IRC in #snapcraft on Freenode.
