.. 4892.md

.. _override-build-steps:

Override build steps
====================

You can override and customise steps of a :ref:`part’s lifecycle <parts-lifecycle>` (pull, build, stage, and prime) using *overrides* which are shell scripts directly sourced from :file:`snapcraft.yaml`. These scripts are run with ``/bin/sh``, which by default on Ubuntu is `dash <https://en.wikipedia.org/wiki/Almquist_shell>`__.

An override is declared with the following syntax:

.. code:: yaml

   parts:
     <part name>:
       override-<step name>: <shell script>

You can use a pipe on the first line to declare a multi-line script:

.. code:: yaml

   parts:
     <part name>:
       override-<step name>: |
         <multi-line>
         <shell script>


.. _override-build-steps-overriding-the-pull-step:

Overriding the pull step
------------------------

This can be done by utilising the ``override-pull`` override. Its working directory is the part’s source directory in ``parts/<part name>/src/``. In order to run the default ``pull`` step, call ``snapcraftctl pull`` from within the override itself.

Example
-------

Let’s say you want to patch the source code of the part you’re pulling:

.. code:: yaml

   parts:
     foo:
       plugin: dump
       # ...
       override-pull: |
         snapcraftctl pull
         patch -p1 < $SNAPCRAFT_STAGE/my.patch


.. _override-build-steps-overriding-the-build-step:

Overriding the build step
-------------------------

This can be done by utilising the ``override-build`` override. Its working directory is the part’s base build directory in ``parts/<part name>/build/``. In order to run the default ``build`` step, call ``snapcraftctl build`` from within the override itself.

.. _example-1:

Example
-------

Let’s say the default build/install process ends up installing files with absolute paths in them, which need to be fixed up to look inside the snap:

.. code:: yaml

   parts:
     foo:
       plugin: dump
       # ...
       override-build: |
         snapcraftctl build
         sed -i 's|/usr/bin|$SNAP/usr/bin|g' $SNAPCRAFT_PART_INSTALL/my-bin-artifact.sh


.. _override-build-steps-overriding-the-stage-step:

Overriding the stage step
-------------------------

This can be done by utilising the ``override-stage`` override. Its working directory is the staging area in ``stage/``. In order to run the default ``stage`` step, call ``snapcraftctl stage`` from within the override itself.

.. _example-2:

Example
-------

Let’s say you wanted to tweak a file installed by another part:

.. code:: yaml

   parts:
     foo:
       plugin: dump
       # ...
       after: [other-part]
       override-stage: |
         snapcraftctl stage
         sed -i 's|/usr/bin|$SNAP/usr/bin|g' other/parts/file


.. _override-build-steps-overriding-the-prime-step:

Overriding the prime step
-------------------------

This can be done by utilising the ``override-prime`` override. Its working directory is the primeing area in ``prime/``. In order to run the default ``prime`` step, call ``snapcraftctl prime`` from within the override itself.

.. _example-3:

Example
-------

Let’s say you wanted to compile gsetting schemas for the entire priming area

.. code:: yaml

   parts:
     foo:
       plugin: nil
       after: [all, other, parts]
       override-prime: |
         snapcraftctl prime
         glib-compile-schemas usr/share/glib-2.0/schemas
