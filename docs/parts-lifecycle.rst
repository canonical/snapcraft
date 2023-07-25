.. 12231.md

.. _parts-lifecycle:

Parts lifecycle
===============

Parts, alongside :ref:`plugins <snapcraft-plugins>`, are a key component in any :ref:`Snapcraft <snapcraft-overview>` project.

See :ref:`Adding parts <adding-parts>` for a general overview of what parts are and how to use them and :ref:`Scriptlets <override-build-steps>` for details on how they can be scripted outside of :file:`snapcraft.yaml`.

All parts within a project, by means of the logic encoded the plugins they’re using, all go through the same series of steps. Knowing these steps, and which directories are used for each step, can help when creating more advanced snaps, and when troubleshooting build issues.


.. _parts-lifecycle-steps:

Lifecycle steps
~~~~~~~~~~~~~~~

The steps a part goes through are as follows:

#. **pull**: downloads or otherwise retrieves the components needed to build the part. You can use the :ref:`source-* keywords <snapcraft-parts-metadata-source>` of a part to specify which components to retrieve. If ``source`` points to a git repository, for example, the pull step will clone that repository.
#. **build**: constructs the part from the previously pulled components. The :ref:`plugin <snapcraft-plugins>` of a part specifies how it is constructed. The :ref:`meson plugin <the-meson-plugin>`, for example, executes ``meson`` and ``ninja`` to compile source code. Each part is built in a separate directory, but it can use the contents of the staging area if it specifies a dependency on other parts using the ``after`` keyword. See `Step dependencies <parts-lifecycle-step-dependencies_>`__ for more information.
#. **stage**: copies the built components into the staging area. This is the first time all the different parts that make up the snap are actually placed in the same directory. If multiple parts provide the same file with differing contents, you will get a conflict. You can avoid these conflicts by using the :ref:`stage keyword <snapcraft-parts-metadata-stage>` to enable or block files coming from the part. You can also use this keyword to filter out files that are not required in the snap itself, for example build files specific to a single part.
#. **prime**: copies the staged components into the priming area, to their final locations for the resulting snap. This is very similar to the stage step, but files go into the priming area instead of the staging area. The ``prime`` step exists because the staging area might still contain files that are required for the build but not for the snap. For example, if you have a part that downloads and installs a compiler, then you stage this part so other parts can use the compiler during building. You can then use the ``prime`` filter keyword to make sure that it doesn’t get copied to the priming area, so it’s not taking up space in the snap. Some extra checks are also run during this step to ensure that all dependencies are satisfied for a proper run time. If confinement was set to ``classic``, then files will be scanned and, if needed, patched to work with this confinement mode.

Finally, **snap** takes the entire contents of the ``prime`` directory and packs it into :ref:`a snap <the-snap-format>`.

Each of these lifecycle steps can be run from the command line, and the command can be part specific or apply to all parts in a project.

#. :command:`snapcraft pull [<part-name>]`
#. :command:`snapcraft build [<part-name>]`
#. :command:`snapcraft stage [<part-name>]`
#. :command:`snapcraft prime [<part-name>]`
#. :command:`snapcraft snap` or :command:`snapcraft`

Note that each command also executes the previous lifecycle steps, so :command:`snapcraft` executes all the lifecycle steps chained together.

To access the part environment at any stage, add the ``--shell`` argument. For example, ``snapcraft prime --shell`` will run up to the *prime* step and open a shell. See :ref:`Iterating over a build <iterating-over-a-build>` for more details.


.. _parts-lifecycle-step-dependencies:

Step dependencies
~~~~~~~~~~~~~~~~~

Each lifecycle step depends on the completion of the previous step for that part, so to reach a desired step, all prior steps need to have successfully run. By default, :command:`snapcraft` runs the same lifecycle step of all parts before moving to the next step. However, you can change this behavior using the ``after`` keyword in the definition of a part in :file:`snapcraft.yaml`. This creates a dependency chain from one part to another.

.. code:: yaml

    grv:
       plugin: go
       go-channel: 1.11/stable
       after:
         - libgit2

In the above example, the part named ``grv`` will be built after the part named ``libgit2`` has been successfully built *and* staged.


.. _parts-lifecycle-overriding-steps:

Overriding a step
~~~~~~~~~~~~~~~~~

Each plugin defines the default actions that happen during a step. This behavior can be changed in two ways.

-  By using ``override-<step-name>`` in :file:`snapcraft.yaml`. See :ref:`Overriding steps <override-build-steps>` for more details.
-  By using a local plugin. This can inherit the parent plugin or scaffolding from the original. See :ref:`Local plugins <writing-local-plugins>` for more details.

See :ref:`Parts environment variables <parts-environment-variables>` for a list of part-specific environment variables that can be accessed to help build a part.


.. _parts-lifecycle-parts-directories:

Parts directories
~~~~~~~~~~~~~~~~~

When running through its build steps, a part will use different working directories. These closely follow the step names for the lifecycle.

+----------------------------+-----------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| Environment variable       | Directory                                     | Purpose                                                                                                                                                                   |
+============================+===============================================+===========================================================================================================================================================================+
| ``SNAPCRAFT_PART_SRC``     | **parts/<part-name>/src**                     | the location of the source during the *pull* step                                                                                                                         |
+----------------------------+-----------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``SNAPCRAFT_PART_BUILD``   | **parts/<part-name>/build**                   | the working directory during the *build* step                                                                                                                             |
+----------------------------+-----------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``SNAPCRAFT_PART_INSTALL`` | **parts/<part-name>/install**                 | contains the results of the *build* step and the stage packages.                                                                                                          |
+----------------------------+-----------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``SNAPCRAFT_STAGE``        | **stage**                                     | shared by all parts, this directory contains the development libraries, headers, and other components (e.g.; pkgconfig files) that need to be accessible from other parts |
+----------------------------+-----------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``SNAPCRAFT_PRIME``        | **prime**                                     | shared by all parts, this directory holds the final components for the resulting snap.                                                                                    |
+----------------------------+-----------------------------------------------+---------------------------------------------------------------------------------------------------------------------------------------------------------------------------+

The following table gives an overview of which directories each step uses. The directories are specified by their environment variables.

+-----------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| Step                              | Explanation                                                                                                                                                                      |
+===================================+==================================================================================================================================================================================+
| **pull**                          | downloads and retrieves the sources specified by the :ref:`source <snapcraft-parts-metadata-source>` key and puts them in SNAPCRAFT_PART\_\ **SRC**                              |
+-----------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| **build**                         | builds the sources in SNAPCRAFT_PART\_\ **BUILD** and places the result in SNAPCRAFT_PART\_\ **INSTALL**                                                                         |
+-----------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| **organize**                      | renames built files in SNAPCRAFT_PART\_\ **INSTALL**                                                                                                                             |
+-----------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| **stage**                         | copies built files from SNAPCRAFT_PART\_\ **INSTALL** to the shared SNAPCRAFT\_\ **STAGE**                                                                                       |
+-----------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| **prime**                         | copies the *staged* files from the shared SNAPCRAFT\_\ **STAGE** to the shared SNAPCRAFT\_\ **PRIME**                                                                            |
+-----------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| **snap**                          | packs contents of SNAPCRAFT\_\ **PRIME** into a snap and puts the snap in SNAPCRAFT_PROJECT_DIR                                                                                  |
+-----------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
