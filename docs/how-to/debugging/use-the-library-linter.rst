.. _how-to-use-the-library-linter:

Use the library linter
======================

The following guidelines describe how to address issues flagged by the library linter.

To resolve a missing dependency, add the missing package to the part's
``stage-packages`` key.

To resolve an unused library:

.. list-table::
   :header-rows: 1
   :widths: 1 3

   * - Library type
     - Resolution

   * - Dynamic linking
     - If the ``stage-packages`` key of the part contains the unused library and no
       other entries, remove the library from the key. If the library contains other
       assets the snap needs, then instead move it to the part's ``stage`` key and
       prefix it with a minus sign (``-``). It's then excluded from the stage step of
       the build.

   * - Static linking
     - Static linking libraries must only be present at build time. So, the part should
       list the libnrary in its ``build-packages`` key, not ``stage-packages``. Again,
       if the library contains other necessary assets for the snap, then move it into
       the part's ``stage`` key and prefix it with a minus sign (``-``).

   * - Dynamic loading
     - Snapcraft may falsely flag dynamic loading libraries as unused. In this case,
       don't change its declaration in the recipe. Moving or removing it has a high risk
       of causing your app to malfunction at runtime. Instead, list it in the
       `lint.ignore.<linter> key <https://snapcraft.io/docs/linters>`_ to suppress the
       warning for this library.


See `Build and staging dependencies
<https://snapcraft.io/docs/build-and-staging-dependencies>`_ for further details about
the ``stage-packages`` key.
