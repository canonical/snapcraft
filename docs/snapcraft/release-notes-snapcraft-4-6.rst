.. 24083.md

.. _release-notes-snapcraft-4-6:

Release notes: Snapcraft 4.6
============================

Snapcraft 4.6 is a feature-packed release, including:

- new extensions
- *core20* support for additional extensions and plugins
- a new login mechanism.
- plenty of bug fixes and smaller updates

For general details, including installation instructions, see `Snapcraft overview <https://snapcraft.io/docs/snapcraft-overview>`__, or take a look at `Snapcraft release notes <https://snapcraft.io/docs/snapcraft-release-notes>`__ for other *Snapcraft* releases.

Login mechanism
---------------

A new option, ``--experimental-login`` can now be used when using ``snapcraft login`` or ``snapcraft export-login`` and when signing assertions (see :ref:`Create a developer account <create-a-developer-account>`).

Using this option will trigger a web based authentication flow. To go back to the previous login method you must first ``snapcraft logout``.

Conda plugin
------------

The :ref:`conda <the-conda-plugin>` plugin has been ported to :ref:`core20 <base-snaps>`. These are the available plugin options:

-  **conda-packages** (list of strings) List of *conda* packages to install.
-  **conda-python-version** (string) The Python version to use for the *conda* packages. Python version major and minor version (e.g. 3.8).
-  **conda-miniconda-version** (string) The version of `Miniconda <https://docs.conda.io/en/latest/miniconda.html>`__ to bootstrap. Defaults to the latest release.

Package Repositories
--------------------

The road to making this feature stable is closer, although a breaking change lands with 4.6 for this experimental feature. Keys are now using the suffix and not prefix of the key id.

See :ref:`Package repositories <snapcraft-package-repositories>` for further details.

Metadata
--------

Snapcraft is now aware of the existence of ``kernel.yaml`` for snaps of type ``kernel``.

The ``install-mode`` option for applications is now supported with this release.

Extensions
----------

Gnome 3.38
~~~~~~~~~~

The :ref:`gnome-3-38 extension <the-gnome-3-38-extension>` is now considered stable.

KDE Neon
~~~~~~~~

The :ref:`KDE Neon extension <the-kde-neon-extension>` now supports ``core20`` as an experimental extension.

Flutter
~~~~~~~

New variants of the :ref:`Flutter extension <the-flutter-extension>` are now available for *stable* and *beta*. The same documentation applies as for the master and dev variant.

Full list of changes
--------------------

-  package-repositories: use last 8 characters of key id for .asc `@cjp256 <https://github.com/cjp256>`__ (`#3486 <https://github.com/snapcore/snapcraft/pull/3486>`__)
-  build(deps): bump lxml from 4.6.2 to 4.6.3 `@dependabot <https://github.com/dependabot>`__ (`#3485 <https://github.com/snapcore/snapcraft/pull/3485>`__)
-  Support install-mode option for apps `@cmatsuoka <https://github.com/cmatsuoka>`__ (`#3482 <https://github.com/snapcore/snapcraft/pull/3482>`__)
-  requirements: use PyNaCl 1.3.0 and ensure is compiled on linux `@sergiusens <https://github.com/sergiusens>`__ (`#3483 <https://github.com/snapcore/snapcraft/pull/3483>`__)
-  tests: crystal 1.0.0 requires shard.lock `@sergiusens <https://github.com/sergiusens>`__ (`#3484 <https://github.com/snapcore/snapcraft/pull/3484>`__)
-  porting conda plugin from v1 to v2 so we can use it in core20 `@ycheng <https://github.com/ycheng>`__ (`#3457 <https://github.com/snapcore/snapcraft/pull/3457>`__)
-  project loader: ensure all key assets are utilized `@cjp256 <https://github.com/cjp256>`__ (`#3364 <https://github.com/snapcore/snapcraft/pull/3364>`__)
-  Candid bakery `@sergiusens <https://github.com/sergiusens>`__ (`#3473 <https://github.com/snapcore/snapcraft/pull/3473>`__)
-  extensions: add core20 support to kde-neon `@sergiusens <https://github.com/sergiusens>`__ (`#3462 <https://github.com/snapcore/snapcraft/pull/3462>`__)
-  ci: add requirements for snapcraft legacy in spread `@sergiusens <https://github.com/sergiusens>`__ (`#3478 <https://github.com/snapcore/snapcraft/pull/3478>`__)
-  ci: reduce amount of artifacts to upload for spread `@sergiusens <https://github.com/sergiusens>`__ (`#3476 <https://github.com/snapcore/snapcraft/pull/3476>`__)
-  godeps: set default channel to 1.15/stable `@cjp256 <https://github.com/cjp256>`__ (`#3475 <https://github.com/snapcore/snapcraft/pull/3475>`__)
-  spread tests: pin go for v1 plugin snaps `@cjp256 <https://github.com/cjp256>`__ (`#3477 <https://github.com/snapcore/snapcraft/pull/3477>`__)
-  Add flutter-stable and -beta extension variants `@MarcusTomlinson <https://github.com/MarcusTomlinson>`__ (`#3471 <https://github.com/snapcore/snapcraft/pull/3471>`__)
-  storeapi: move http client and auth to http_clients package `@sergiusens <https://github.com/sergiusens>`__ (`#3472 <https://github.com/snapcore/snapcraft/pull/3472>`__)
-  store: do not unnecessarily catch/rewrite exceptions `@cjp256 <https://github.com/cjp256>`__ (`#3466 <https://github.com/snapcore/snapcraft/pull/3466>`__)
-  ci: run spread store tests when secret is available `@sergiusens <https://github.com/sergiusens>`__ (`#3470 <https://github.com/snapcore/snapcraft/pull/3470>`__)
-  gitignore: sort list `@cjp256 <https://github.com/cjp256>`__ (`#3467 <https://github.com/snapcore/snapcraft/pull/3467>`__)
-  repo: introduce DebPackage class to standardize package name parsing `@cjp256 <https://github.com/cjp256>`__ (`#3460 <https://github.com/snapcore/snapcraft/pull/3460>`__)
-  store: set auth headers when using login –with `@sergiusens <https://github.com/sergiusens>`__ (`#3468 <https://github.com/snapcore/snapcraft/pull/3468>`__)
-  meta: add support for ``kernel.yaml`` for kernel snaps `@mvo5 <https://github.com/mvo5>`__ (`#3464 <https://github.com/snapcore/snapcraft/pull/3464>`__)
-  extensions: make GNOME 3.38 stable `@sergiusens <https://github.com/sergiusens>`__ (`#3427 <https://github.com/snapcore/snapcraft/pull/3427>`__)
-  requirements: pip freeze `@sergiusens <https://github.com/sergiusens>`__ (`#3458 <https://github.com/snapcore/snapcraft/pull/3458>`__)
-  storeapi: decouple auth and API `@sergiusens <https://github.com/sergiusens>`__ (`#3452 <https://github.com/snapcore/snapcraft/pull/3452>`__)
