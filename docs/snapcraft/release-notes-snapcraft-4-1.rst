.. 18769.md

.. _release-notes-snapcraft-4-1:

Release notes: Snapcraft 4.1
============================

The team behind Snapcraft is pleased to announce the release of `Snapcraft 4.1 <https://github.com/snapcore/snapcraft/releases/tag/4.1>`__.

The most interesting feature in this release is the addition a :ref:`flutter <the-flutter-plugin>` plugin to help with the creation of snaps for Flutter based applications, and a flutter extension to help handle their dependencies.

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

Full list of changes
--------------------

The issues and features worked on for Snapcraft 4.1 are reflected in the following change list:

-  flutter v1 plugin: new plugin for flutter `@sergiusens <https://github.com/sergiusens>`__ (`#3192 <https://github.com/snapcore/snapcraft/pull/3192>`__)
-  flutter v1 plugin: pull from source-subdir if set `@sergiusens <https://github.com/sergiusens>`__ (`#3200 <https://github.com/snapcore/snapcraft/pull/3200>`__)
-  extensions: introduce flutter-dev `@sergiusens <https://github.com/sergiusens>`__ (`#3199 <https://github.com/snapcore/snapcraft/pull/3199>`__)
-  extensions: introduce flutter-master `@sergiusens <https://github.com/sergiusens>`__ (`#3195 <https://github.com/snapcore/snapcraft/pull/3195>`__)
-  riscv64 support `@xnox <https://github.com/xnox>`__ (`#3186 <https://github.com/snapcore/snapcraft/pull/3186>`__)
-  plugins: add support for local v2 plugins (core20) `@cjp256 <https://github.com/cjp256>`__ (`#3118 <https://github.com/snapcore/snapcraft/pull/3118>`__)
-  snap: support for lzo as a compression target `@sergiusens <https://github.com/sergiusens>`__ (`#3189 <https://github.com/snapcore/snapcraft/pull/3189>`__)

Maintenance
-----------

-  pyinstaller: workaround pkg_resources issue `@sergiusens <https://github.com/sergiusens>`__ (`#3201 <https://github.com/snapcore/snapcraft/pull/3201>`__)
-  extensions: export content snap egl vendor dir `@sergiusens <https://github.com/sergiusens>`__ (`#3190 <https://github.com/snapcore/snapcraft/pull/3190>`__)
-  cli: use snap pack instead of mksquashfs `@sergiusens <https://github.com/sergiusens>`__ (`#3173 <https://github.com/snapcore/snapcraft/pull/3173>`__)
-  extensions: plug the opengl interface for GNOME `@sergiusens <https://github.com/sergiusens>`__ (`#3193 <https://github.com/snapcore/snapcraft/pull/3193>`__)

Bug Fixes
---------

-  link_or_copy: do not try to create hardlinks to symlinks. `@hpoul <https://github.com/hpoul>`__ (`#3174 <https://github.com/snapcore/snapcraft/pull/3174>`__)
-  cli: allow promoting from edge without –yes `@sergiusens <https://github.com/sergiusens>`__ (`#3185 <https://github.com/snapcore/snapcraft/pull/3185>`__)
-  maven plugin: improve error message when target libs are not found. `@edumucelli <https://github.com/edumucelli>`__ (`#3179 <https://github.com/snapcore/snapcraft/pull/3179>`__)
-  cli: unset false boolean flags in environment `@cjp256 <https://github.com/cjp256>`__ (`#3196 <https://github.com/snapcore/snapcraft/pull/3196>`__)
-  cli: use maxval of UnknownLength for pack progress `@sergiusens <https://github.com/sergiusens>`__ (`#3187 <https://github.com/snapcore/snapcraft/pull/3187>`__)
-  build providers: check revision before switching `@sergiusens <https://github.com/sergiusens>`__ (`#3184 <https://github.com/snapcore/snapcraft/pull/3184>`__)

Specifications and Documentation
--------------------------------

-  extensions: introduce flutter-master `@sergiusens <https://github.com/sergiusens>`__ (`#3195 <https://github.com/snapcore/snapcraft/pull/3195>`__)

Tooling
-------

-  tools: fix environment-setup to work on aarch64 `@cjp256 <https://github.com/cjp256>`__ (`#3176 <https://github.com/snapcore/snapcraft/pull/3176>`__)


