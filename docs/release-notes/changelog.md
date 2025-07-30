---
tocdepth: 2
---

(changelog)=

# Changelog

<!-- release template:

## X.Y.Z (YYYY-MMM-DD)

### Core

for everything related to the lifecycle of packing a snap

#### Bases

##### <coreXX>

(order from newest base to oldest base)

#### Plugins

##### <plugin>

#### Extensions

##### <extension>

#### Metadata

#### Sources

#### Components

### Command line

for command line and UX changes

### Linter

### Init

### Metrics

### Names

### Remote build

### Store

### Documentation

For a complete list of commits, check out the `X.Y.Z`_ release on GitHub. -->

(7.5.8_changelog)=

## 7.5.8 (2024-Oct-24)

### Core

- Fix a regression where Snapcraft would fail to run on some architectures due to a
  `cryptography` dependency that attempted to load legacy algorithms ([#5077]).

For a complete list of commits, check out the [7.5.8] release on GitHub.

(7.5.7_changelog)=

## 7.5.7 (2024-Oct-03)

### Core

- Fix a bug where parallel installations of Snapcraft would not work if the Snapcraft
  snap was installed from the store ([#4683], [#4927]).

For a complete list of commits, check out the [7.5.7] release on GitHub.

(7.5.6_changelog)=

## 7.5.6 (2024-Aug-15)

### Core

- Configure icon for snaps without apps ([#4950])
- Update spread tests in hotfix branch([#4954])

For a complete list of commits, check out the [7.5.6] release on GitHub.

[#4950]: https://github.com/canonical/snapcraft/issues/4950
[#4954]: https://github.com/canonical/snapcraft/issues/4954
[#4683]: https://github.com/canonical/snapcraft/issues/4683
[#4927]: https://github.com/canonical/snapcraft/issues/4927
[#5077]: https://github.com/canonical/snapcraft/issues/5077
[7.5.6]: https://github.com/canonical/snapcraft/releases/tag/7.5.6
[7.5.7]: https://github.com/canonical/snapcraft/releases/tag/7.5.7
[7.5.8]: https://github.com/canonical/snapcraft/releases/tag/7.5.8
