.. 20810.md

.. _release-notes-snapcraft-4-4:

Release notes: Snapcraft 4.4
============================

The team behind Snapcraft is pleased to announce the release of `Snapcraft 4.4 <https://github.com/snapcore/snapcraft/releases/tag/4.4>`__.

Highlights for this release include:

-  improved font generation performance when :ref:`desktop extensions <supported-extensions>`
-  updates to package repository definitions and behaviour
-  metrics for :ref:`Progressive Releases <progressive-releases>`
-  many small but significant bug fixes

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

New Features
------------

Command Line
~~~~~~~~~~~~

Custom CA certificates can now be added to the build environments created by Snapcraft by using ``--add-ca-certificates`` with the :ref:`lifecycle <parts-lifecycle>` related commands.

Extensions
~~~~~~~~~~

This release includes a new mechanism to pre-generate font caches with :ref:`desktop-related extensions <supported-extensions>`. This enables system font cache generation at install time, isolated from the potentially (ABI) incompatible host-generated cache.

To benefit from this feature, extension snaps will need to be rebuilt. More details about this feature can be found in the `specification <https://github.com/snapcore/snapcraft/blob/master/specifications/desktop-extensions-font-hook.org>`__.

Package repositories
~~~~~~~~~~~~~~~~~~~~

The *experimental* :ref:`package repositories <snapcraft-package-repositories>` has the following changes:

-  the undocumented ``$SNAPCRAFT_APT`` variables have been removed.
-  improved error handling and schema validation.

Additionally, the package repository `specification <https://github.com/snapcore/snapcraft/blob/master/specifications/package-repositories.org>`__ has been finalised, hopefully opening a path to remove the *experimental* flag for this feature.

Updates to the Store
~~~~~~~~~~~~~~~~~~~~

Progressive Releases
^^^^^^^^^^^^^^^^^^^^

Metrics have been added to our progressive releases feature, as defined by the `specification <https://github.com/snapcore/snapcraft/blob/master/specifications/progressive-releases.org>`__. Input on this feature is welcome on the `forum <https://forum.snapcraft.io/new-topic?title=Progressive%20Releases%20Feedback&category=snapcraft>`__.

See :ref:`Progressive releases <progressive-releases>` for more details.

Snap Revisions
^^^^^^^^^^^^^^

Snapcraft’s ``revisions`` command is now internally using a new endpoint to provide much better results:

-  table headers have changed to better represent architectures
-  channels are now fully qualified (i.e.; ``<track/risk/branch>`` )
-  comma separated column entries no longer have a space separating each item

Refer to the `specification <https://github.com/snapcore/snapcraft/blob/master/specifications/history-to-releases.org>`__ for further details.

Full list of changes
--------------------

The issues and features worked on for Snapcraft 4.4 are reflected in the following change list:

New Features
~~~~~~~~~~~~

-  cli: remove spaces from progressive metrics `@sergiusens <https://github.com/sergiusens>`__ (`#3335 <https://github.com/snapcore/snapcraft/pull/3335>`__)
-  storeapi: add releases endpoint `@sergiusens <https://github.com/sergiusens>`__ (`#3311 <https://github.com/snapcore/snapcraft/pull/3311>`__)
-  cli: provide option to install ca certs into build environment `@cjp256 <https://github.com/cjp256>`__ (`#3224 <https://github.com/snapcore/snapcraft/pull/3224>`__)
-  Add PYTHONPATH to environment (fixes #1893262) `@hellsworth <https://github.com/hellsworth>`__ (`#3270 <https://github.com/snapcore/snapcraft/pull/3270>`__)
-  snap packaging: fetch remote icons configured via appstream `@cjp256 <https://github.com/cjp256>`__ (`#3241 <https://github.com/snapcore/snapcraft/pull/3241>`__)
-  pluginhandler: support using patchelf on strict snaps `@kyrofa <https://github.com/kyrofa>`__ (`#3277 <https://github.com/snapcore/snapcraft/pull/3277>`__)
-  gnome extensions: multiarch bindtextdomain.so support `@galgalesh <https://github.com/galgalesh>`__ (`#3127 <https://github.com/snapcore/snapcraft/pull/3127>`__)
-  db: introduce generalized datastore `@cjp256 <https://github.com/cjp256>`__ (`#3238 <https://github.com/snapcore/snapcraft/pull/3238>`__)
-  meta: write stubs for command-chain using hooks when needed `@sergiusens <https://github.com/sergiusens>`__ (`#3296 <https://github.com/snapcore/snapcraft/pull/3296>`__)
-  cli: support snap –output `@cjp256 <https://github.com/cjp256>`__ (`#3297 <https://github.com/snapcore/snapcraft/pull/3297>`__)
-  extensions: configure hook for fonts `@sergiusens <https://github.com/sergiusens>`__ (`#3299 <https://github.com/snapcore/snapcraft/pull/3299>`__)
-  storeapi: add support for reporting status of progressive releases `@maxiberta <https://github.com/maxiberta>`__ (`#3306 <https://github.com/snapcore/snapcraft/pull/3306>`__)
-  package repositories: improve error handling `@cjp256 <https://github.com/cjp256>`__ (`#3334 <https://github.com/snapcore/snapcraft/pull/3334>`__)
-  spread tests: move package-repositories test snaps into own dir `@cjp256 <https://github.com/cjp256>`__ (`#3331 <https://github.com/snapcore/snapcraft/pull/3331>`__)

Maintenance
~~~~~~~~~~~

-  cli: update revisions to use releases API `@sergiusens <https://github.com/sergiusens>`__ (`#3329 <https://github.com/snapcore/snapcraft/pull/3329>`__)
-  storeapi: remove bindings for history `@sergiusens <https://github.com/sergiusens>`__ (`#3332 <https://github.com/snapcore/snapcraft/pull/3332>`__)
-  v1 plugins: lock godep’s dependencies `@cjp256 <https://github.com/cjp256>`__ (`#3285 <https://github.com/snapcore/snapcraft/pull/3285>`__)
-  readme: remove link to Google+ `@timsueberkrueb <https://github.com/timsueberkrueb>`__ (`#3292 <https://github.com/snapcore/snapcraft/pull/3292>`__)
-  storeapi: drop arch requirement for get_channel_mapping() `@cjp256 <https://github.com/cjp256>`__ (`#3301 <https://github.com/snapcore/snapcraft/pull/3301>`__)
-  build(deps-dev): bump junit from 3.8.1 to 4.13.1 in /tests/spread/plugins/v1/maven/snaps/legacy-maven-hello/my-app `@dependabot <https://github.com/dependabot>`__ (`#3316 <https://github.com/snapcore/snapcraft/pull/3316>`__)
-  build(deps-dev): bump junit from 3.8.1 to 4.13.1 in /tests/spread/plugins/v1/maven/snaps/maven-hello/my-app `@dependabot <https://github.com/dependabot>`__ (`#3315 <https://github.com/snapcore/snapcraft/pull/3315>`__)
-  spread tests: introduce electron-builder test `@cjp256 <https://github.com/cjp256>`__ (`#3312 <https://github.com/snapcore/snapcraft/pull/3312>`__)
-  unit tests: fix runtests.sh not filtering tests when passed a subdirectory `@maxiberta <https://github.com/maxiberta>`__ (`#3305 <https://github.com/snapcore/snapcraft/pull/3305>`__)
-  electron-builder spread test: sync expected snapcraft.yaml `@cjp256 <https://github.com/cjp256>`__ (`#3323 <https://github.com/snapcore/snapcraft/pull/3323>`__)
-  package repositories: drop $SNAPCRAFT_APT_HOST_ARCH variable `@cjp256 <https://github.com/cjp256>`__ (`#3322 <https://github.com/snapcore/snapcraft/pull/3322>`__)
-  package repositories: drop $SNAPCRAFT_APT_RELEASE variable `@cjp256 <https://github.com/cjp256>`__ (`#3328 <https://github.com/snapcore/snapcraft/pull/3328>`__)
-  flutter tests: updated for latest embedder `@kenvandine <https://github.com/kenvandine>`__ (`#3310 <https://github.com/snapcore/snapcraft/pull/3310>`__)
-  lxd unit tests: simplify command checking pattern `@cjp256 <https://github.com/cjp256>`__ (`#3326 <https://github.com/snapcore/snapcraft/pull/3326>`__)

Bug Fixes
~~~~~~~~~

-  package repositories: fix case where formats is empty `@cjp256 <https://github.com/cjp256>`__ (`#3330 <https://github.com/snapcore/snapcraft/pull/3330>`__)
-  meta: add error check for “command not found” `@cjp256 <https://github.com/cjp256>`__ (`#3321 <https://github.com/snapcore/snapcraft/pull/3321>`__)
-  snapcraftctl: add checks for empty string for set-version & set-grade `@cjp256 <https://github.com/cjp256>`__ (`#3325 <https://github.com/snapcore/snapcraft/pull/3325>`__)
-  pluginhandler: properly handle snapcraftctl errors `@cjp256 <https://github.com/cjp256>`__ (`#3317 <https://github.com/snapcore/snapcraft/pull/3317>`__)
-  schema: add regex to validate description is non-empty `@cjp256 <https://github.com/cjp256>`__ (`#3303 <https://github.com/snapcore/snapcraft/pull/3303>`__)
-  set ROS_PYTHON_VERSION for rosdep `@artivis <https://github.com/artivis>`__ (`#3324 <https://github.com/snapcore/snapcraft/pull/3324>`__)
-  Set ROS_VERSION for rosdep in plugins v1 `@artivis <https://github.com/artivis>`__ (`#3313 <https://github.com/snapcore/snapcraft/pull/3313>`__)
-  repo: install requested build-package versions `@cjp256 <https://github.com/cjp256>`__ (`#3221 <https://github.com/snapcore/snapcraft/pull/3221>`__)
-  project loader: install dirmngr prior to configuring package repositories `@cjp256 <https://github.com/cjp256>`__ (`#3294 <https://github.com/snapcore/snapcraft/pull/3294>`__)
-  build providers: fix issues running on Windows `@sergiusens <https://github.com/sergiusens>`__ (`#3289 <https://github.com/snapcore/snapcraft/pull/3289>`__)
-  cmake v2 plugin: add help for cmake generators `@sergiusens <https://github.com/sergiusens>`__ (`#3288 <https://github.com/snapcore/snapcraft/pull/3288>`__)
-  setup.py: assert with helpful error when unable to determine version `@cjp256 <https://github.com/cjp256>`__ (`#3307 <https://github.com/snapcore/snapcraft/pull/3307>`__)

Specifications and Documentation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  specifications: v1 history to v2 releases `@sergiusens <https://github.com/sergiusens>`__ (`#3304 <https://github.com/snapcore/snapcraft/pull/3304>`__)
-  specification: desktop extension font hook `@sergiusens <https://github.com/sergiusens>`__ (`#3295 <https://github.com/snapcore/snapcraft/pull/3295>`__)
-  specifications: finalization of package repositories spec `@cjp256 <https://github.com/cjp256>`__ (`#3333 <https://github.com/snapcore/snapcraft/pull/3333>`__) 
