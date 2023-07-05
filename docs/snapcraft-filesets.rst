.. 8973.md

.. _snapcraft-filesets:

Snapcraft filesets
==================

A *fileset* is used within :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` to represent a group of files, or a single file, when :ref:`creating a snap <snapcraft-overview>`.

.. note::
          Filesets do not currently work when building snaps with ``core22`` :ref:`base snaps <base-snaps>` and Snapcraft 7.x .

They’re useful when moving files into the *stage* and *prime* areas of the build process (see :ref:`Snapcraft lifecycle <parts-lifecycle>` for more details) as they can be referenced individually, rather than always having to construct a list of filenames.

A fileset is implemented as a YAML map between the fileset name (the key) and a list of corresponding filenames for each fileset. This list can be built from any of the following:

-  individual files, eg. ``[ bin/dnsmasq ]``
-  subdirectory paths, eg. ``[ etc ]``
-  wildcard globs, eg. ``[ usr/* ]``
-  globstar globs, eg. ``[ lib/**/*.so* ]``

The ``*`` (asterisk) wildcard glob returns all the files in that path. Conversely, adding an initial ``-`` (dash) will exclude the files in that path. For example, you could add ``usr/local/*`` then remove ``usr/local/man/*`` with the following:

.. code:: yaml

   filesets:
      allbutman: [ usr/local/*, -usr/local/man/* ]

Filenames are relative to the part install directory, eg. ``parts/<part-name>/install``.

If you have used the :ref:`organize <snapcraft-parts-metadata>` keyword to rename files from your snapcraft.yaml part, your fileset will be built from filenames *after* they’re renamed.

Conflicting rules example
-------------------------

Snapcraft *will* attempt to aggregate conflicting rules from different filesets. For example, take the following directory and file structure:

.. code:: bash

   adir
   ├── adirthat
   ├── adirthis
   └── bdir
       ├── bdirthat
       └── bdirthis

The following fileset definition will not stage ``adir/bdir/*`` despite its specific inclusion under ``adir/*``:

.. code:: yaml

   filesets:
     exclude-dir: [ -adir/bdir/* ]
     include-dir: [ adir/* ]
   stage:
     - $include-dir
     - $exclude-dir

In the above example, using the excluding syntax ``adir/bdir`` instead of ``adir/bdir/*`` would exclude both the ``bdir`` directory **and** its contents, rather than excluding only the contents, retaining the empty ``bdir`` itself.

This is how the above will be staged within the snap:

::

   adir
   ├── adirthat
   ├── adirthis
   └── bdir

If, however, we remove the ``exclude-dir: [ -adir/bdir/* ]`` fileset definition, all the files and directories beneath *adir* will be staged in the snap:

::

   adir
   ├── adirthat
   ├── adirthis
   └── bdir
       ├── bdirthat
       └── bdirthis

Relevant Snapcraft source code
------------------------------

Check out `the \_organize_filesets function in snapcraft_legacy/internal/pluginhandler/init.py <https://github.com/snapcore/snapcraft/blob/7b848f76debfa2cb020308c5b908eb570d06c0b9/snapcraft_legacy/internal/pluginhandler/__init__.py#L1306-L1355>`__
