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

    - base: core18
    + base: core20

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
