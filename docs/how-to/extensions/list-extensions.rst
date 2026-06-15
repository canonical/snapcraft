.. _how-to-list-extensions:

List extensions
===============

Different versions of Snapcraft and its various cores support different extensions. To
list all extensions supported by the installed version of Snapcraft, run:

.. code-block:: bash

    snapcraft extensions

The output includes all extensions, both stable and experimental, without
differentiating them.

.. vale off

.. terminal::
    :output-only:

    Extension name          Supported bases
    ----------------------  ------------------------------
    dotnet10              core24
    dotnet8               core24
    dotnet9               core24
    env-injector          core24
    gnome                 core22, core24
    gpu                   core22, core24, core26
    kde-neon              core22, core24
    kde-neon-6            core22, core24
    kde-neon-qt6          core22, core24
    [...]

.. vale on

For the full reference for this command, see :ref:`ref_commands_extensions`.
