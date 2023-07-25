.. 10725.md

.. _release-notes-snapcraft-3-3:

Release notes: Snapcraft 3.3
============================

These are the release notes for `Snapcraft 3.3 <https://github.com/snapcore/snapcraft/releases/tag/3.3>`__.

For general details, including installation instructions, see :ref:`Snapcraft overview <snapcraft-overview>`, or take a look at :ref:`Snapcraft release notes <snapcraft-release-notes>` for other *Snapcraft* releases.

New *core* features
-------------------

base: core
~~~~~~~~~~

Support for ``base: core`` has been added.

This enables you to take advantage of the many new Snapcraft 3 features while using 16.04 as a :ref:`base <base-snaps>`. Previously, you needed to use the 18.04 base.

Alternate directory for snapcraft.yaml
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``build-aux/snap`` is now supported as an alternative directory for :file:`snapcraft.yaml` and its assets (i.e.; hooks, gui, …).

To avoid confusion, snapcraft now display what directory it is picking for assets, depending on where the :file:`snapcraft.yaml` is found. It will only pick ``build-aux/snap`` for assets if the :file:`snapcraft.yaml` file is found in that path.

String validation
~~~~~~~~~~~~~~~~~

Snapcraft now produces a better error when the type detected for the version *string* is not a *string*.

Plugins
-------

python
~~~~~~

A minor fix to bring *rebuilding* capabilities to projects using the python plugin.

This means the days of seeing messages like the following should be long gone:

.. code:: bash

   You must give at least one requirement to install (maybe you meant "pip install ...")

python and go
~~~~~~~~~~~~~

Expanded schema errors for users of the Go and Python, allowing for early discovery of non-valid uses of these plugins. Most importantly, this eliminates the cryptic error during build time when not using a combination of the ``source`` and ``<plugin-name>-packages`` keywords.

nodejs
~~~~~~

Entries of type *string* are now supported in ``package.json`` for the ``bin`` keyword (previously, the plugin could only parse dictionary entries), this means that constructs from ``package.json``, such as the following, will be parsed and interpreted correctly by Snapcraft:

.. code:: json

   {
     "name": "unnamed",
     "version": "1.0",
     "description": "Using string bin entries.",
     "main": "index.js",
     "bin": "bin/index.js",
   }

Store
-----

Register against a specific store
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Brand stores are a commercial feature of the Snap Store, and it’s now possible to register your snap with one of these stores.

The following syntax is now allowed as part of the ``register`` command:

.. code:: bash

   $ snapcraft register [--store <store>] <snap-name>

Full list of changes
--------------------

The issues and features worked on for 3.3 can be seen on the `3.3 <https://bugs.launchpad.net/snapcraft/+milestone/3.3>`__ launchpad milestone which are reflected in the following change list:

**Sergio Schvezov**

-  many: support for “base: core” in snapcraft.yaml (`#2499 <https://github.com/snapcore/snapcraft/pull/2499>`__) (LP: #1819290)
-  python plugin: graceful ret when no packages set (`#2498 <https://github.com/snapcore/snapcraft/pull/2498>`__) (LP: #1794216)
-  many: support the use of build-aux/snap (`#2496 <https://github.com/snapcore/snapcraft/pull/2496>`__) (LP: #1805219)
-  nodejs plugin: support for type str bin entries (`#2501 <https://github.com/snapcore/snapcraft/pull/2501>`__) (LP: #1817553)
-  store: support registering to a specific store (`#2479 <https://github.com/snapcore/snapcraft/pull/2479>`__) (LP: #1820107)
-  meta: fix management of snap/local (`#2502 <https://github.com/snapcore/snapcraft/pull/2502>`__)
-  tests: improve login pexpect errors
-  tests: correctly retry registers
-  build providers: enhance provider errors (`#2508 <https://github.com/snapcore/snapcraft/pull/2508>`__) (LP: #1821217)
-  build providers: improve handling in snap logic (`#2507 <https://github.com/snapcore/snapcraft/pull/2507>`__) (LP: #1820864)
-  tests: filter per arch and fix snap build deps

**Claudio Matsuoka**

-  sources: handle network request errors (`#2494 <https://github.com/snapcore/snapcraft/pull/2494>`__)
-  store: handle invalid snap file errors (`#2492 <https://github.com/snapcore/snapcraft/pull/2492>`__)
-  tests: fix multipass error handling spread test (`#2491 <https://github.com/snapcore/snapcraft/pull/2491>`__)
-  plugins: improve python and go schema validation (`#2473 <https://github.com/snapcore/snapcraft/pull/2473>`__) (LP: #1806055)
-  cli: disable raven if not running from package (`#2503 <https://github.com/snapcore/snapcraft/pull/2503>`__)

**Facundo Batista**

-  schema: better ‘version’ error messages: wrong type and incorrect length (`#2497 <https://github.com/snapcore/snapcraft/pull/2497>`__)
   (LP: #1815812)


