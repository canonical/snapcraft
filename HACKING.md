# Snapcraft

## Setting up a development environment

We want to make sure everyone develops using a consistent base, to ensure that these instructions rely on LXD (use whatever is convenient as long as you do not stray away from an Ubuntu LTS base)

Clone the snapcraft repository and its submodules and make it your working directory:

```shell
git clone https://github.com/canonical/snapcraft.git --recurse-submodules
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

### Tooling

We use a large number of tools for our project. Most of these are installed for
you with tox, but you'll need to install:

- Python 3.12 (default on Ubuntu 24.04) with setuptools.
- [tox](https://tox.wiki) version 3.8 or later
- [pyright](https://github.com/microsoft/pyright) (also available via snap: `snap install pyright`)
- [ruff](https://github.com/astral/ruff) (also available via snap: `snap install ruff`)
- [ShellCheck](https://www.shellcheck.net/) (also available via snap: `snap install shellcheck`)

### Testing

See the [Testing guide](TESTING.md).

### Enabling debug output

Given that the `--debug` option in snapcraft is reserved for project specific debugging, enabling for the `logger.debug` calls is achieved by setting the "SNAPCRAFT_ENABLE_DEVELOPER_DEBUG" environment variable to a truthful value. Snapcraft's internal tools, e.g.; `snapcraftctl` should pick up this environment variable as well.

## Documentation

### Build

To render the documentation as HTML in `docs/_build`, run:

```shell
tox run -e build-docs
```

> **Important**
>
> Interactive builds are currently defective and cause an infinite loop. [This GitHub issue](https://github.com/sphinx-doc/sphinx/issues/11556#issuecomment-1667451983) posits that this is caused by by pages referencing each other.

If you prefer to compose pages interactively, you can host the documentation on a local server:

```shell
tox run -e autobuild-docs
```

You can reach the interactive site at http://127.0.0.1:8080 in a web browser.

### Test

The documentation Makefile provided by the [Sphinx Starter Pack](https://github.com/canonical/sphinx-docs-starter-pack) provides a number of natural language checks such as style guide adherence, inclusive words, and product terminology, however they currently aren't configured correctly for Snapcraft. Instead, you can validate for basic language and syntax using two of the development tests.

To check for syntax errors in documentation, run:

```shell
tox run -e lint-docs
```

For a rudimentary spell check, you can use codespell:

```shell
tox run -e lint-codespell
```

## Evaluating pull requests

Oftentimes all you want to do is see if a given pull request solves the issue you were having. To make this easier, a snap is published for `amd64` on a channel named `latest/edge/pr-<PR number>` where `PR number` is the number of the pull request.

For feature branches, a snap is published for `amd64` on a channel named `latest/edge/<branch name>`. For example, a branch named `feature/offline-mode` would be available on the channel `latest/edge/offline-mode`.

## Reaching out

We'd love the help!

- Submit pull requests against [snapcraft](https://github.com/canonical/snapcraft/pulls)
- Make sure to read the [contribution guide](CONTRIBUTING.md)
- Find us under the snapcraft category of the forum https://forum.snapcraft.io
- Discuss with us using IRC in #snapcraft on Freenode.
