.. meta::
    :description: How to change the base of a snap. Switch between bases like core24 to core26. This guide describes the different requirements of each migration path.

.. _how-to-change-bases:

Change bases
============

A :ref:`base snap <base-snaps>` is a special kind of snap that provides a runtime
environment with a minimal set of libraries common to most applications. They're
transparent to users, but they need to be carefully considered when building a snap.

Each base snap is built from a :ref:`corresponding Ubuntu LTS <reference-bases>`
release. Migrating a snap from one base to the next gives the snap access to newer
packages, extended support, and the latest Snapcraft features, including plugins and
:ref:`extensions <reference-extensions>`.

The complexity of the migration process is directly linked to both dependencies in the
snap's project file and the base snap versions being migrated between.

At its simplest, migrating from one base snap to another requires only that the ``base``
key is updated:

.. code-block:: diff
    :caption: snapcraft.yaml

    - base: core24
    + base: core26

But further changes will most likely be needed. These will depend on the
original base and the packages that are bundled with the application.


Update from an old or absent base
---------------------------------

Migrating a snap from having no base, or ``base: core``, to core18 or core20,
for example, is a more involved process than going from core18 to core20.

This is because building a snap with an old base causes Snapcraft to operate in
compatibility mode.

Compatibility mode is essentially an older (2.43-era) version of Snapcraft, and will
lose the functionality of newer releases.


Core migration guides
---------------------

Our core migration how-to guides provide detailed instructions and examples to help you
transition your snap between specific bases.

.. toctree::
    :titlesonly:
    :maxdepth: 1

    change-from-core18-to-core20
    change-from-core20-to-core22
    change-from-core22-to-core24
    change-from-core24-to-core26


Base changelogs
---------------

Each base snap contains a changelog at ``/usr/share/doc/ChangeLog``
that summarizes the package updates bundled in that release,
along with the aggregated updates from previous releases.
In the examples below, replace ``<base>`` with the name of
your base snap (for example, core24).


When the snap is already installed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If the base snap is already installed on the system, the changelog is available
at ``/snap/<base>/current/usr/share/doc/ChangeLog``.

You can list all installed revisions and versions of a base with ``snap list --all <base>``.
To inspect a specific revision, replace ``current`` with
the revision number. For example, ``/snap/<base>/<revision>/usr/share/doc/ChangeLog``.


Download a base
~~~~~~~~~~~~~~~

You can download a base without installing it, then extract the
changelog from it:

.. code-block:: bash

    snap download <base>
    unsquashfs -d <base>-unpacked <base>_*.snap usr/share/doc/ChangeLog

The changelog is then available at ``<base>-unpacked/usr/share/doc/ChangeLog``.
