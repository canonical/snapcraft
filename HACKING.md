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
make setup
```

### Tooling

We use a large number of tools for our project. Most of these are installed for
you with `make setup`, but you'll need to install Python 3.12 separately.

### Testing

By default, our CI suite will run both unit and integration testing on every PR. Generally speaking, it's good to run unit tests before pushing any code anyways as they are quick and great for catching bugs early. Integration tests, on the other hand, are quite heavy and slow to run locally. Due to this, we only recommend running the integration tests locally if there is an obvious need, such as debugging an integration test that failed in CI.

#### Unit testing

For unit testing, use the provided make recipes:

```shell
make test       # All unit tests
make test-fast  # Only the fast tests
make test-slow  # Only the slow tests
```

#### Integration testing

For integration testing, Snapcraft uses [Spread](https://github.com/canonical/spread). Spread is a system for distributing tests and executing them in different backends, in parallel.

To test with Spread, first fetch the Spread testing tools:

```shell
git submodule update --init
```

Next, build Snapcraft into a snap:

```shell
snapcraft pack
```

Then, move the resulting snap into the tests directory:

```shell
mv *.snap tests/
```

Next, install Spread with Go:

```shell
go install github.com/snapcore/spread/cmd/spread@latest
```

Ensure that the installation added the `$HOME/go/bin` directory to the `$PATH` environment variable.

Then, you can run the integration tests using a local LXD backend with:

```shell
spread -v lxd:
```

You can also run them in Google Cloud if you have a Google Cloud credentials JSON file. In order to do this, run:

```shell
SPREAD_GOOGLE_KEY={credentials_path} spread -v google:
```

### Enabling debug output

Given that the `--debug` option in snapcraft is reserved for project specific debugging, enabling for the `logger.debug` calls is achieved by setting the "SNAPCRAFT_ENABLE_DEVELOPER_DEBUG" environment variable to a truthful value. Snapcraft's internal tools, e.g.; `snapcraftctl` should pick up this environment variable as well.

## Documentation

### Build

To render the documentation as HTML in `docs/_build`, run:

```shell
make docs
```

> **Important**
>
> Interactive builds are currently defective and cause an infinite loop. [This GitHub issue](https://github.com/sphinx-doc/sphinx/issues/11556#issuecomment-1667451983) posits that this is caused by by pages referencing each other.

If you prefer to compose pages interactively, you can host the documentation on a local server:

```shell
make auto-docs
```

You can reach the interactive site at http://127.0.0.1:8080 in a web browser.

### Test

The documentation Makefile provided by the [Sphinx Starter Pack](https://github.com/canonical/sphinx-docs-starter-pack) provides a number of natural language checks such as style guide adherence, inclusive words, and product terminology, however they currently aren't configured correctly for Snapcraft. Instead, you can validate for basic language and syntax using two of the development tests.

To check for syntax errors in documentation, run:

```shell
make lint-docs
```

For a rudimentary spell check, you can use codespell:

```shell
make lint-codespell
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
