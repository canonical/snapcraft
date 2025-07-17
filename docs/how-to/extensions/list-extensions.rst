.. _how-to-list-extensions:

List extensions
===============

Different versions of Snapcraft and its various cores support different extensions. To
list all extensions supported by the installed version of Snapcraft, run:

.. code-block:: bash

    snapcraft list-extensions

The output includes all extensions, both stable and experimental, without
differentiating them.

.. terminal::

    Extension name          Supported bases
    ----------------------  ------------------------------
    env-injector            core24
    flutter-beta            core18
    flutter-dev             core18
    flutter-master          core18
    flutter-stable          core18
    gnome                   core22, core24
    gnome-3-28              core18
    gnome-3-34              core18
    gnome-3-38              core20
    kde-neon                core18, core20, core22, core24
    [...]

For the full reference for this command, see :ref:`ref_commands_list-extensions`.
