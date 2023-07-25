.. 8336.md

.. _snapcraft-parts-metadata:

Snapcraft parts metadata
========================

The main building blocks of a snap are *parts*. They are used to declare pieces of code that will be pulled into your snap package. The *parts* keys and values in :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` detail how parts are configured and built by the *snapcraft* command.

   See :ref:`Snapcraft top-level metadata <snapcraft-top-level-metadata>` and :ref:`Snapcraft apps and services metadata <snapcraft-app-and-service-metadata>` for details on how apps and parts are configured within :file:`snapcraft.yaml`.

parts
-----

Type: ``dict``

A set of independent building blocks.

These independent building blocks are known as *parts*, and consist of either code or pre-built packages.

parts.<part-name>
-----------------

Type: ``dict``

The name of the part building block.

``<part-name``> represents the specific name of a building block which can be then referenced by the command-line tool (i.e. :command:`snapcraft`).

Keys for parts
--------------

The following are keys that can be used within **parts.** (for example, ``parts.<part-name>.plugin``):

after
~~~~~

Type: ``list[string]``

Ensures that all the <part-names> listed in ``after`` are staged before this part begins its :ref:`lifecycle <parts-lifecycle-steps>`.


.. _snapcraft-parts-metadata-build-attributes:

build-attributes
~~~~~~~~~~~~~~~~

Type: ``enum``

A list of named attributes to modify the behaviour of plugins.

Supported attributes:

-  ``debug``: Plugins that support the concept of build types build in Release mode by default. Setting the ‘debug’ attribute requests that they instead build in debug mode.
-  ``keep-execstack``: Do not remove the “executable stack” bit from ELF files.
-  ``no-patchelf``: Do not patch ELF files, even when Snapcraft believes it is required (e.g. for classic snaps)
-  ``enable-patchelf``: Do patch ELF files, even when Snapcraft does not believe it’s required (e.g. for strict snaps)
-  ``no-install``: Do not run the install target provided by the plugin’s build system. *(Only supported by the* :ref:`kbuild plugin <the-kbuild-plugin>` *)*

For more information, refer to the output of ``snapcraft help plugins``.


.. _snapcraft-parts-metadata-build-environment:

build-environment
~~~~~~~~~~~~~~~~~

Type: Array

A list of environment variable assignments that are applied during the build step, `it is exported in order which allows for later values to override (or modify) earlier values. <https://github.com/snapcore/snapcraft/pull/2322>`__

This entry supports additional syntax, for more information refer to :ref:`Advanced grammar <snapcraft-advanced-grammar>`.

.. code:: yaml

   parts:
     _part_name_:
       build-environment:
         - LANG: C.UTF-8
         - LC_ALL: C.UTF-8

build-packages
~~~~~~~~~~~~~~

Type: ``list[string]``

A list of packages required to build a snap.

Packages are installed using the host’s package manager, such as ``apt`` or ``dnf``, and are required for <part-name> to build correctly. This entry supports additional syntax, for more information refer to :ref:`Advanced grammar <snapcraft-advanced-grammar>`.

Example: ``[ libssl-dev, libssh-dev, libncursesw5-dev]``


.. _snapcraft-parts-metadata-build-snaps:

build-snaps
~~~~~~~~~~~

Type: ``list[string]``

A list of snap names to install that are necessary to build ``<part-name>``.

If a specific channel is required, the syntax is of the form ``<snap-name>/<channel>``. This entry supports additional syntax, for more information refer to :ref:`Advanced grammar <snapcraft-advanced-grammar>`

Example: ``build-snaps: [go/1.13/stable]``


.. _snapcraft-parts-metadata-disable-parallel:

disable-parallel
~~~~~~~~~~~~~~~~

Type: ``boolean``

Whether to disable parallelism for the build plugins.

filesets
~~~~~~~~

Type: ``list[string]``

A key to represent a group of files or a single file.

See :ref:`Snapcraft filesets <snapcraft-filesets>` for further details.

organize
~~~~~~~~

Type: ``dict``

A map of files to rename.

In the key/value pair, the key represents the path of a file inside the part and the value represents how the file is going to be staged.

Example: ``bin/snapcraftctl: bin/scriptlet-bin/snapcraftctl``


.. _snapcraft-parts-metadata-override-build:

override-build
~~~~~~~~~~~~~~

Type: ``multiline string``

Replaces a plugin’s default *build* process with a script.

The shell script defined here replaces the :ref:`build <parts-lifecycle-steps>` step of the plugin, defined in ``parts.<part-name>.plugin``. The working directory is the base build directory for the given part. The defined script is run with ``/bin/sh`` and ``set -e``. A set of :ref:`Environment Variables <environment-variables>` will be available to the script.

To run Snapcraft’s original build implementation from within *override-build*, run ``snapcraftctl build``. This can be run before or after any custom script or omitted entirely.


.. _snapcraft-parts-metadata-override-prime:

override-prime
~~~~~~~~~~~~~~

Type: ``multiline string``

Replaces a plugin’s default *prime* process with a script.

The shell script defined here replaces the :ref:`prime <parts-lifecycle-steps>` step of the plugin, defined in ``parts.<part-name>.plugin``. The working directory is the base prime directory for the given part. The defined script is run with ``/bin/sh`` and ``set -e``. A set of :ref:`Environment Variables <environment-variables>` will be available to the script.

To run Snapcraft’s original prime step implementation from within *override-prime*, run ``snapcraftctl prime``. This can be run before or after any custom script or omitted entirely.


.. _snapcraft-parts-metadata-override-pull:

override-pull
~~~~~~~~~~~~~

Type: ``multiline string``

Replaces a plugin’s default *pull* process with a script.

The shell script defined here replaces the :ref:`pull <parts-lifecycle-steps>` step of the plugin, defined in ``parts.<part-name>.plugin``. The working directory is the base pull directory for the given part. The defined script is run with ``/bin/sh`` and ``set -e``. A set of :ref:`Environment Variables <environment-variables>` will be available to the script.

To run Snapcraft’s original pull stage implementation from within *override-pull*, run ``snapcraftctl pull``. This can be run before or after any custom script or omitted entirely.


.. _snapcraft-parts-metadata-override-stage:

override-stage
~~~~~~~~~~~~~~

Type: ``multiline string``

Replaces a plugin’s default *stage* process with a script.

The shell script defined here replaces the :ref:`stage <parts-lifecycle-steps>` step of the plugin, defined in ``parts.<part-name>.plugin``. The working directory is the base stage directory for the given part. The defined script is run with ``/bin/sh`` and ``set -e``. A set of :ref:`Environment Variables <environment-variables>` will be available to the script.

To run Snapcraft’s original stage implementation from within *override-stage*, run ``snapcraftctl stage``. This can be run before or after any custom script or omitted entirely.

parse-info
~~~~~~~~~~

Type: ``list[string]``

Defines content to adopt when using external metadata.

Each entry is a relative path to a :ref:`supported metadata file <using-external-metadata>` from the part source, build or install directory (:ref:`SNAPCRAFT_PART_SRC, SNAPCRAFT_PART_BUILD, SNAPCRAFT_PART_INSTALL <parts-lifecycle-parts-directories>`).

See :ref:`Using external metadata <using-external-metadata>` for more details.

plugin
~~~~~~

Type: ``string``

The plugin to drive the build process.

Every part drives its build through a plugin, this entry declares the plugin that will drive the build process for ``<part-name>``. Refer to :ref:`snapcraft plugins <snapcraft-plugins>` for more information on the available plugins and the specific attributes they add to the ``parts.<part-name>.`` namespace.

prepare (deprecated)
~~~~~~~~~~~~~~~~~~~~

Type: ``multiline string``

Runs a script before the plugin’s :ref:`build <parts-lifecycle-steps>` step.

The script is run before the build step defined for ``parts.<part-name>.plugin`` starts. The working directory is the base build directory for the given part. The defined script is run with ``/bin/sh`` and ``set -e``. A set of :ref:`Environment Variables <environment-variables>` will be available to the script.

   ⚠ The release of :ref:`Snapcraft 3.0 <release-notes-snapcraft-3-0>` made this key obsolete. Use `override-build <snapcraft-parts-metadata-override-build_>`__ instead.

prime
~~~~~

Type: ``list[string]``

A list of files from <part-name> to :ref:`prime <parts-lifecycle-steps>`.

Rules applying to the list here are the same as those of filesets. Referencing of fileset keys is done with a ``$`` prefixing the fileset key, which will expand with the value of such key.


.. _snapcraft-parts-metadata-source:

source
~~~~~~

Type: ``string``

A URL or path to a source tree to build.

This can be a local path or remote and can refer to a directory tree, a compressed archive, or a revision control repository. This entry supports additional syntax, for more information refer to :ref:`Advanced grammar <snapcraft-advanced-grammar>`

source-branch
~~~~~~~~~~~~~

Type: ``string``

Work on a specific branch for source repositories under version control.

source-checksum
~~~~~~~~~~~~~~~

Type: ``string``

Used when ``source`` represents a file.

Takes the syntax ``<algorithm>/<digest>``, where ``<algorithm>`` can be any of: ``md5``, ``sha1``, ``sha224``, ``sha256``, ``sha384``, ``sha512``, ``sha3_256``, ``sha3_384`` or ``sha3_512``. When set, the source is cached for multiple uses in different snapcraft projects.

source-commit
~~~~~~~~~~~~~

Type: ``string``

Work on a specific commit for source repositories under version control.

source-depth
~~~~~~~~~~~~

Type: ``integer``

Depth of history for sources using version control.

Source repositories under version control are cloned or checked out with full history. Specifying a depth will truncate the history to the specified number of commits.

source-subdir
~~~~~~~~~~~~~

Type: ``string``

A path within the ``source`` to set as the working directory when building. The build will *not* be able to access files outside of this location, such as one level up.

source-submodules
~~~~~~~~~~~~~~~~~

Type: ``dict``

Configure which submodules to fetch from the source tree in snapcraft.yaml with ``source-submodules: <list-of-submodules>``

When **source-submodules** is defined, only the listed submodules are fetched:

.. code:: yaml

   parts:
     git-test:
       plugin: dump
       source-type: git
       source: git@github.com...
       source-submodules:
         - submodule_1
         - dir1/submodule_2

If **source-submodules** is defined and the list is empty, no submodules are fetched:

.. code:: yaml

   parts:
     git-test:
       plugin: dump
       source-type: git
       source: git@github.com...
       source-submodules: []

If source-submodules is not defined, all submodules are fetched (default behaviour).

source-tag
~~~~~~~~~~

Type: ``string``

Work on a specific tag for source repositories under version control.

source-type
~~~~~~~~~~~

Type: ``enum``

Used when the type of ``source`` entry cannot be detected.

Can be one of the following: ``[bzr|deb|git|hg|local|mercurial|rpm|subversion|svn|tar|zip|7z]``


.. _snapcraft-parts-metadata-stage:

stage
~~~~~

Type: ``list[string]``

A list of files from <part-name> to stage.

Rules applying to the list here are the same as those of filesets. Referencing of fileset keys is done with a ``$`` prefixing the fileset key, which will expand with the value of such key.

stage-packages
~~~~~~~~~~~~~~

Type: ``list[string]``

A list of packages required at runtime by a snap.

Packages are required by <part-name> to run. They are fetched using the host’s package manager, such as ``apt`` or ``dnf``, and are unpacked into the snap being built. This entry supports additional syntax, for more information refer to :ref:`Advanced grammar <snapcraft-advanced-grammar>`.

Example: ``[python-zope.interface, python-bcrypt]``

stage-snaps
~~~~~~~~~~~

Type: ``list[string]``

A list of snaps required at runtime by a snap.

Snaps are required by <part-name> to run. They are fetched using ``snap download``, and are unpacked into the snap being built. This entry supports additional syntax, for more information refer to :ref:`Advanced grammar <snapcraft-advanced-grammar>`.

Example: ``[hello, black/latest/edge]``
