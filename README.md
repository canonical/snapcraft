<img src="https://dashboard.snapcraft.io/site_media/appmedia/2018/04/Snapcraft-logo-bird.png" alt="Snapcraft logo" style="height: 128px; display: block">

# Snapcraft

[![Snapcraft][snapcraft-badge]][snapcraft-site]
[![Documentation Status][rtd-badge]][rtd-latest]
[![Spread tests][gha-spread-badge]][gha-spread]
[![Codecov Status][codecov-badge]][codecov-status]
[![Ruff status][ruff-badge]][ruff-site]

**Snapcraft** is the command-line build tool for packaging and distributing software and
apps in the snap container format. It solves the problems of dependency management and
architecture support by bundling all of a software's libraries into the container
itself, and gives developers a way to package any app, program, toolkit, or library for
all major Linux distributions and IoT devices.

## Basic usage

A snap's build configuration is stored in simple language as a project file called
`snapcraft.yaml`, making it easy to add as a new package format to your existing code
base.

From the root of the code base of any software project, Snapcraft creates a minimal
`snapcraft.yaml` with:

```bash
snapcraft init
```

After you add all your project's build and runtime details to the project file, bundle
your project into a snap with:

```bash
snapcraft pack
```

Your project can be registered on public and private app stores, including the Snap
Store:

```bash
snapcraft register
```

Snap versions and revisions, including parallel releases, are published to the store
with:

```bash
snapcraft upload
```

If you're interested in learning more about the Snapcraft commands and how to compose a
project file, try [crafting your first
snap](https://documentation.ubuntu.com/snapcraft/stable/tutorials/craft-a-snap).

## Installation

Snapcraft is available on all major Linux distributions, Windows, and macOS.

Snapcraft itself has first-class support as a snap. On snap-ready systems, you can install it on the command line with:

```bash
sudo snap install snapcraft --classic
```

For complete installation, you need an additional Linux container tool. Snapcraft can
also be installed as a traditional package on many popular Linux repositories. If you
need help with either, the documentation covers how to [set up
Snapcraft](https://documentation.ubuntu.com/snapcraft/stable/how-to/setup/set-up-snapcraft).

## Documentation

The [Snapcraft documentation](https://documentation.ubuntu.com/snapcraft/stable)
provides guidance and learning material about the full process of building a project
file, debugging snaps, resolving interfaces, the command reference, and much more.

## Community and support

We are a growing community of crafters who build snaps for all Linux software.

Ask your questions about Snapcraft and what's on the horizon, and see who's working on
what in the [Snapcraft Forum](https://forum.snapcraft.io) and on the [Snapcraft Matrix
channel](https://matrix.to/#/#snapcraft:ubuntu.com).

You can report any issues or bugs on the project's [GitHub
repository](https://github.com/canonical/snapcraft/issues).

Snapcraft is covered by the [Ubuntu Code of
Conduct](https://ubuntu.com/community/ethos/code-of-conduct).

## Contribute to Snapcraft

Snapcraft is open source and part of the Canonical family. We would love your help.

If you're interested, start with the [contribution guide](CONTRIBUTING.md).

We welcome any suggestions and help with the docs. The [Canonical Open Documentation
Academy](https://github.com/canonical/open-documentation-academy) is the hub for doc
development, including Snapcraft docs. No prior coding experience is required.

## License and copyright

Snapcraft is released under the [GPL-3.0 license](LICENSE).

© 2015-2025 Canonical Ltd.

[snapcraft-badge]: https://snapcraft.io/snapcraft/badge.svg
[snapcraft-site]: https://snapcraft.io/snapcraft
[rtd-badge]: https://readthedocs.com/projects/canonical-snapcraft/badge/?version=latest
[rtd-latest]: https://documentation.ubuntu.com/snapcraft/latest/?badge=latest
[gha-spread-badge]: https://github.com/canonical/snapcraft/actions/workflows/spread-manual.yaml/badge.svg?branch=main
[gha-spread]: https://github.com/canonical/snapcraft/actions/workflows/spread-manual.yaml
[ruff-badge]: https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json
[ruff-site]: https://github.com/astral-sh/ruff
[codecov-badge]: https://codecov.io/github/canonical/snapcraft/coverage.svg?branch=master
[codecov-status]: https://codecov.io/github/canonical/snapcraft?branch=master
