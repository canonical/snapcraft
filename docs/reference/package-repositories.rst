.. _reference-package-repositories:

Package repositories
====================

When building a snap and constructing a part, package dependencies are listed as either
package names or snaps for the snap's build environment. This is covered in `Build and
staging dependencies <https://snapcraft.io/docs/build-and-staging-dependencies>`_.

For a default Snapcraft installation, the build environment is invariably some version
of Ubuntu. Consequently, dependencies are listed using their APT package names and are
retrieved from the set of repositories officially supported by the distribution.

Using the ``package-repositories`` key, it's possible to add your own APT repositories
as sources for the ``build-packages`` and ``stage-packages`` keys, including those
listed on a Personal Package Archive (PPA), which serves personally hosted non-standard
packages.

For more information on how to configure package repositories, see the `Craft Archives
documentation`_.

.. _Craft Archives documentation: https://canonical-craft-archives.readthedocs-hosted.com/en/latest/reference/repo_properties/
