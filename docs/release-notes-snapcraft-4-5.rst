.. 22786.md

.. _release-notes-snapcraft-4-5:

Release notes: Snapcraft 4.5
============================

The team behind Snapcraft is pleased to announce the release of `Snapcraft 4.5 <https://github.com/snapcore/snapcraft/releases/tag/4.5>`__.

Highlights for this release include:

-  path property for exact paths when specifying repositories
-  a new gnome-3-38 extension which uses gnome-3-38-2004 and core20
-  enable 7z, bzr, hg, svn, zip for non-linux sources
-  port of the *qmake* plugin to v2 of the plugin spec

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

Full list of changes
--------------------

The issues and features worked on for Snapcraft 4.5 are reflected in the following change list:

-  repo: key management refactor `@cjp256 <https://github.com/cjp256>`__ (`#3359 <https://github.com/snapcore/snapcraft/pull/3359>`__)
-  repo: default to target arch for stage package cache `@cjp256 <https://github.com/cjp256>`__ (`#3416 <https://github.com/snapcore/snapcraft/pull/3416>`__)
-  package repositories: introduce path property for exact paths `@cjp256 <https://github.com/cjp256>`__ (`#3336 <https://github.com/snapcore/snapcraft/pull/3336>`__)
-  project loader, schema: add advanced grammar support for build-environment `@cjp256 <https://github.com/cjp256>`__ (`#3350 <https://github.com/snapcore/snapcraft/pull/3350>`__)
-  project: enable experimental target-arch support for core20 `@cjp256 <https://github.com/cjp256>`__ (`#3410 <https://github.com/snapcore/snapcraft/pull/3410>`__)
-  project: always set target arch even if not cross compiling `@cjp256 <https://github.com/cjp256>`__ (`#3418 <https://github.com/snapcore/snapcraft/pull/3418>`__)
-  build(deps): bump lxml from 4.5.0 to 4.6.2 `@dependabot <https://github.com/dependabot>`__ (`#3404 <https://github.com/snapcore/snapcraft/pull/3404>`__)
-  Allow validation assertions to be revoked `@nessita <https://github.com/nessita>`__ (`#3417 <https://github.com/snapcore/snapcraft/pull/3417>`__)
-  Add new gnome-3-38 extension which uses gnome-3-38-2004 and core20 `@kenvandine <https://github.com/kenvandine>`__ (`#3407 <https://github.com/snapcore/snapcraft/pull/3407>`__)
-  project loader: export SNAPCRAFT_TARGET_ARCH in build environment `@cjp256 <https://github.com/cjp256>`__ (`#3414 <https://github.com/snapcore/snapcraft/pull/3414>`__)
-  grammar: ensure all dictionary primitives are captured `@cjp256 <https://github.com/cjp256>`__ (`#3412 <https://github.com/snapcore/snapcraft/pull/3412>`__)
-  Fix a few licenses in ros-related test files `@artivis <https://github.com/artivis>`__ (`#3409 <https://github.com/snapcore/snapcraft/pull/3409>`__)
-  repo: only install build packages marked for installation `@cjp256 <https://github.com/cjp256>`__ (`#3411 <https://github.com/snapcore/snapcraft/pull/3411>`__)
-  sources: enable 7z, bzr, hg, svn, zip for non-linux `@cjp256 <https://github.com/cjp256>`__ (`#3369 <https://github.com/snapcore/snapcraft/pull/3369>`__)
-  project loader: advanced grammar support for lists `@cjp256 <https://github.com/cjp256>`__ (`#3360 <https://github.com/snapcore/snapcraft/pull/3360>`__)
-  elf: extract defined symbol versions `@jhenstridge <https://github.com/jhenstridge>`__ (`#3408 <https://github.com/snapcore/snapcraft/pull/3408>`__)
-  [feature] ROS plugins v2 out of source tree builds `@artivis <https://github.com/artivis>`__ (`#3405 <https://github.com/snapcore/snapcraft/pull/3405>`__)
-  pluginhandler: do not walk symlinks for include filesets `@cjp256 <https://github.com/cjp256>`__ (`#3406 <https://github.com/snapcore/snapcraft/pull/3406>`__)
-  storeapi: remove unused MissingSnapdError `@sergiusens <https://github.com/sergiusens>`__ (`#3403 <https://github.com/snapcore/snapcraft/pull/3403>`__)
-  cli: add missing quote key creation hint `@sergiusens <https://github.com/sergiusens>`__ (`#3402 <https://github.com/snapcore/snapcraft/pull/3402>`__)
-  yaml_utils: promote module to a package `@cjp256 <https://github.com/cjp256>`__ (`#3385 <https://github.com/snapcore/snapcraft/pull/3385>`__)
-  ci: uprev pyinstaller and switch timestamp server `@cjp256 <https://github.com/cjp256>`__ (`#3401 <https://github.com/snapcore/snapcraft/pull/3401>`__)
-  autotools v2 plugin: support autogen.sh and bootstrap `@sergiusens <https://github.com/sergiusens>`__ (`#3398 <https://github.com/snapcore/snapcraft/pull/3398>`__)
-  requirements: uprev python-apt `@sergiusens <https://github.com/sergiusens>`__ (`#3400 <https://github.com/snapcore/snapcraft/pull/3400>`__)
-  cli: do not require snapd deb for assertions `@sergiusens <https://github.com/sergiusens>`__ (`#3399 <https://github.com/snapcore/snapcraft/pull/3399>`__)
-  plugins v2: port the qmake plugin `@jhenstridge <https://github.com/jhenstridge>`__ (`#3391 <https://github.com/snapcore/snapcraft/pull/3391>`__)
-  plugins v2: add support for out of source tree builds `@jhenstridge <https://github.com/jhenstridge>`__ (`#3392 <https://github.com/snapcore/snapcraft/pull/3392>`__) 
