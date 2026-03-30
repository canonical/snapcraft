.. meta::
    :description: How to migrate a snap from core24 to core26.

.. _how-to-change-from-core24-to-core26:

Change from core24 to core26
============================

This is a guide for migrating a snap that uses core24 as its base to core26.

Core26 is built from Ubuntu 26.04 LTS. For most snaps, the migration consists of
checking dependencies and extensions. If your snap has strict confinement and runs
Python code, you need to add a Python runtime.

Stability and workarounds
-------------------------

In the wake of a new core, some of the updated features and packages your snap relies on
will be unstable for a time.

This is to be expected with complex software. A short-term fix or workaround in your
snap's parts, packages, and scriptlets might be needed. But sometimes, a temporary fix
becomes permanent. It's a good practice to explain your workarounds with comments in
your project's ``snapcraft.yaml``.

Update the base
---------------

Start by updating the ``base`` key:

.. code-block:: diff
    :caption: snapcraft.yaml

    - base: core24
    + base: core26

Check extension support
-----------------------

Not all extensions are compatible with core26 at its launch. If your snap uses an
extension, run ``snapcraft extensions`` to see if it's available for core26. If your
snap uses an extension that does not yet support core26, it's best to wait to upgrade.

Update the packages
-------------------

Package names and versions change between Ubuntu releases. If you have parts that
declare :ref:`build-packages <PartSpec.build_packages>` or :ref:`stage-packages
<PartSpec.stage_packages>`, you might need to update those packages in your project file.

Build your snap and note any dependent packages that fail to resolve. Search the
`Ubuntu package archive <https://packages.ubuntu.com>`__ for the equivalent on Ubuntu
26.04, and update ``build-packages`` and ``stage-packages`` accordingly.

A package may have been split, merged, or renamed:

.. code-block:: yaml
    :caption: snapcraft.yaml

    stage-packages:
      - libexample3t64  # renamed from libexample3 in core26

Repackage Python
----------------

Core26 has `updated its Python support commitment
<https://forum.snapcraft.io/t/core26-bundled-python-changes/50178>`__, and doesn't ship
with Python. Going forward, all strictly-confined snaps that run Python must pack a
separate Python binary.

The optimal solution is to define a separate part, and stage the Python runtime inside it:

.. code-block:: yaml
    :caption: snapcraft.yaml
    :emphasize-lines: 2-9,12

    parts:
      python-runtime:
        plugin: nil
        stage-packages:
          - libpython3.14-minimal
          - libpython3.14-stdlib
          - python3.14-minimal
          - python3-venv
          - python3-minimal

      my-app:
        after: [python-runtime]
        source: .
        plugin: python

Replace snapcraftctl
--------------------

With core26, snapcraftctl is replaced with :ref:`craftctl <reference-external-package-scriptlets>`.

In your project file's scriptlets, find and replace all instances of ``snapcraftctl`` with
``craftctl``. For example:


.. code-block:: diff
    :caption: snapcraft.yaml

    - snapcraftctl pull
    + craftctl default
    - snapcraftctl set-grade stable
    + craftctl set grade=stable

Update Spread test systems
--------------------------

Snaps should behave the same regardless of the host operating system, but testing
on Ubuntu 26.04 may expose failures, such as classically-confined snaps incorrectly
accessing host packages.

If your project uses the :ref:`test <ref_commands_test>` command for integration
testing, it's recommended to test your snap with Ubuntu 26.04. Add it to the list of
systems in your project's ``spread.yaml``:

.. code-block:: yaml
    :caption: spread.yaml
    :emphasize-lines: 3

    systems:
      - ubuntu-24.04
      - ubuntu-26.04

Retaining older releases in the Spread configuration is recommended.
