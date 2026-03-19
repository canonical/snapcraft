.. _explanation-parts-lifecycle:

Parts lifecycle
===============

Building a snap involves processing one or more :ref:`explanation-parts`. Each part
follows an ordered series of build steps, which forms the *lifecycle*, ensuring a
coherent and reproducible build process.

This page explains how parts work within Snapcraft -- including their steps, commands,
directories, and dependencies.


Lifecycle steps
---------------

During build, a part goes through five steps:

1. **Pull**. Downloads or otherwise retrieves the components needed to build the part.

   The ``source`` key of a part specifies which components to retrieve. For instance, if
   ``source`` points to a Git repository, the pull step will clone that repository.

2. **Build**. Constructs the part from the previously pulled components.

   The ``plugin`` key of a part specifies how it is constructed. The
   :ref:`craft_parts_meson_plugin`, for example, runs ``meson`` and ``ninja`` to compile
   source code.

   Each part is built in a separate directory, but it can use the contents of the
   staging area if it specifies a dependency on other parts using the ``after`` key. See
   the :ref:`Processing order and dependencies
   <explanation-parts-lifecycle-processing-order>` section for details.

   The ``organize`` key moves and renames files.

3. **Stage**. Copies the built components into the staging area.

   This is the first time all the different parts that make up the snap are actually
   placed in the same directory. It's possible for multiple parts to provide clashing
   file names. The ``stage`` key allows or blocks files coming from the part to
   circumvent such conflicts. It's also used to filter out files that aren't required in
   the snap itself, such as build files specific to a single part.

4. **Prime**. Copies the staged components into the priming area.

   This is very similar to the stage step, but files go into the priming area instead of
   the staging area. The prime step exists because the staging area might still contain
   files that are required for the build but not for the snap. For example, if a part
   downloads and installs a compiler, then this part should be staged so that other
   parts can use the compiler during build. The ``prime`` key could then filter the
   compiler from the priming area, so it doesn't take up space inside the snap.

   The ``permissions`` key controls file permissions.

   Some extra checks are also run during this step to ensure that all dependencies are
   satisfied for a proper run time. If the snap uses classic confinement, then files
   will be scanned and, if needed, patched to work with this confinement mode.

5. **Pack**. Takes the entire contents of the ``prime`` directory and packages it into a
   snap.


Commands
--------

Each step in the lifecycle has a corresponding command in Snapcraft. This allows
developers to manually run through the steps, aiding in debugging and understanding
snaps. The command can be part-specific, or applied to all parts.

The commands are, in order:

1. ``snapcraft pull [<part-name>]``
2. ``snapcraft build [<part-name>]``
3. ``snapcraft stage [<part-name>]``
4. ``snapcraft prime [<part-name>]``
5. ``snapcraft pack``

Each command executes the previous lifecycle steps, so ``snapcraft stage`` runs the pull
and build steps first, while ``snapcraft pack`` chains all the lifecycle steps together,
in order.

To enter the part environment at any stage, add the ``--shell`` argument to these
commands. For example, ``snapcraft prime --shell`` will run up to the prime step and
open a shell. See :ref:`iterate-on-the-build-lifecycle` for more practical details.


.. _explanation-parts-lifecycle-directories:

Directories
-----------

When running through its lifecycle steps, a part will use different working directories.
The directories' names closely follow the steps' names.

.. list-table::
    :header-rows: 1
    :widths: 2 1 3

    * - Directory
      - Environment variable
      - Purpose
    * - ``parts/<part-name>/src``
      - ``CRAFT_PART_SRC``
      - The location of the source after the pull step.
    * - ``parts/<part-name>/build``
      - ``CRAFT_PART_BUILD``
      - The working directory during the build step.
    * - ``parts/<part-name>/install``
      -  ``CRAFT_PART_INSTALL``
      - Contains the results of the build step and the stage packages. It's also the
        directory where the ``organize`` event renames the built files.
    * - ``stage``
      - ``CRAFT_STAGE``
      - Shared by all parts, this directory contains the contents of each part's
        ``CRAFT_PART_INSTALL`` after the stage step. It can contain development
        libraries, headers, and other components (such as pkgconfig files) that need to
        be accessible from other parts.
    * - ``CRAFT_PRIME``
      - ``prime``
      - Shared by all parts, this directory holds the final components after the prime
        step.
    * - The current project's path in the filesystem.
      - ``CRAFT_PROJECT_DIR``
      - Used to access resources from the project's subtree, such as an icon or version
        file.


Overriding a step
-----------------

Each plugin defines the default actions that happen during a step. This behavior can be
changed with the ``override-<step-name>`` key.
:ref:`how-to-customize-the-build-and-part-variables` provides guidance.


.. _explanation-parts-lifecycle-processing-order:

Processing order and dependencies
---------------------------------

Each lifecycle step depends on the completion of the previous step. During build,
Snapcraft walks through one step at a time, processing that step for all parts. Within a
step, the parts are processed in alphabetical order. Only after a step is complete for
every part will it continue to the next step.


Overriding the part order
~~~~~~~~~~~~~~~~~~~~~~~~~

The part order can be overridden by the ``after`` key in the part's definition. The
purpose of the key is to stagger the part order so that interrelated parts can provide
data to each other.

With ``after``, the part order follows modified rules:

- Parts are ordered alphabetically by name, as usual.
- When the build reaches a part that another depends on, the dependent part will only
  start its build and stage steps after the initial part finishes its stage step.
- After a chain of parts completes, the step continues to the next part in alphabetical
  order.


Example 1 -- Default lifecycle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this example, the default order applies. Each step is processed serially, and within
each step the parts are processed in alphabetical order.

.. code-block:: yaml
    :caption: snapcraft.yaml

    parts:
      Alex:
        plugin: go
      Cam:
        plugin: go
      Blair:
        plugin: go


.. terminal::

    Pulling Alex
    Pulling Blair
    Pulling Cam
    Building Alex
    Building Blair
    Building Cam
    Staging Alex
    Staging Blair
    Staging Cam
    ...


Example 2 -- Order override
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml
    :caption: snapcraft.yaml

    parts:
      Alex:
        plugin: go
        after:
          - Cam
      Blair:
        plugin: go
      Cam:
        plugin: go


.. terminal::

    Pulling Cam
    Pulling Alex
    Pulling Blair
    Building Cam
    Skipping pull for Cam (already ran)
    Skipping build for Cam (already ran)
    Staging Cam (required to build 'A')
    Building Alex
    Building Blair
    Skipping stage for Cam (already ran)
    Staging Alex
    Staging Blair
    ...

In the above example, the part named ``A`` is built after the part named ``C`` has been
successfully built and staged.


Lifecycle processing diagram
----------------------------

.. image:: https://assets.ubuntu.com/v1/07d25e64-lifecycle_logic.png
    :alt: A flowchart of the parts lifecycle, showing the processing order and conditional paths.


Learn more
----------

For more information about the parts lifecycle, see the following resources:

- :ref:`reference-part-environment-variables` has a list of part-specific environment
  variables that can be accessed at build time.
- :ref:`Scriptlets <reference-external-package-scriptlets>` has
  more details on how to override steps.
