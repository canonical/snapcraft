.. _how-to-enable-classic-confinement:

Enable classic confinement
==========================

When a snap needs system resources beyond what strict confinement and the available
`interfaces <https://snapcraft.io/docs/supported-interfaces>`_ can provide, it needs
:ref:`classic confinement <explanation-classic-confinement>`.

This page provides guidance on how to enable classic confinement, as well as example
solutions for snaps that have extra plugin-specific configurations to accommodate it.


Set the confinement level
-------------------------

You can enable classic confinement with a single key in the project file:

.. code-block:: yaml
    :caption: snapcraft.yaml

    confinement: classic

When built, the snap now has unsandboxed access to the system, like a traditional app or
package.

Next, :ref:`check for linting errors
<how-to-enable-classic-confinement-identify-problems-linters>` to look for low-hanging
errors that Snapcraft can detect.

Lastly, you may also need to :ref:`provide extra configuration for the main part's
plugin <how-to-enable-classic-confinment-example-project-files>`.


.. _how-to-enable-classic-confinement-request-snap-store:

Request classic confinement on the Snap Store
---------------------------------------------

A classically-confined snap requires approval from the Store team before you can
distribute it on the Snap Store. Obtaining approval takes on average three to five
business days.

`Submit your snap for review
<https://snapcraft.io/docs/reviewing-classic-confinement-snaps>`_ to get approval for
classic confinement.


.. _how-to-enable-classic-confinement-identify-problems-linters:

Identify problems with linters
------------------------------

When you first enable classic confinement, you should run an appropriate :ref:`built-in
linter <reference-linters>` to check for common errors and problems in the snap's files.


.. _how-to-enable-classic-confinment-example-project-files:

Example project files with classic confinement
----------------------------------------------

The following projects are sample solutions for combining classic confinement with
specific plugins. These are only guidelines. Every case is different, so you may need to
make other adjustments to your snap's build to completely confine it.


Autotools plugin
~~~~~~~~~~~~~~~~

`autotools-classic-example
<https://github.com/snapcraft-doc-samples-unofficial/autotools-classic-example>`_ is a
project with a main part that uses the Autotools plugin.

.. dropdown:: autotools-classic-example project file

    .. literalinclude:: ../code/crafting/enable-classic-confinement/example-classic-confinement-autotools.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Make plugin
~~~~~~~~~~~

`makefile-lib-example
<https://github.com/snapcraft-doc-samples-unofficial/makefile-lib-example>`_ is a
project with a main part that uses the Make plugin.

.. dropdown:: makefile-lib-example project file

    .. literalinclude:: ../code/crafting/enable-classic-confinement/example-classic-confinement-make.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


CMake plugin
~~~~~~~~~~~~

`cmake-classic-example <https://github.com/snapcraft-docs/cmake-classic-example>`_ is a
project with a main part that uses the CMake plugin.

.. dropdown:: cmake-classic-example project file

    .. literalinclude:: ../code/crafting/enable-classic-confinement/example-classic-confinement-cmake.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Python plugin
~~~~~~~~~~~~~

`python-ctypes-example <https://github.com/snapcraft-docs/python-ctypes-example>`_ is a
project with a main part that uses the Python plugin.

.. dropdown:: python-ctypes-example project file

    .. literalinclude:: ../code/crafting/enable-classic-confinement/example-classic-confinement-python.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Patch Python ctypes to load system libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If your app uses ctypes to access system libraries, it must be bundled with patched
ctype files. In this sample, both the patch script and the diff are kept in the project
as files for processing during build.

To craft this solution:

1. Copy the `patch script
   <https://github.com/snapcraft-docs/python-ctypes-example/blob/main/snap/local/patch-ctypes.sh>`_
   into ``snap/local/`` and the `diff file
   <https://github.com/snapcraft-docs/python-ctypes-example/blob/main/snap/local/patches/ctypes_init.diff>`_
   into ``snap/local/patches/``.

2. Stage the system libraries in the main part. The core22 base uses Python 3.10, so the
   packages are:

   .. literalinclude:: ../code/crafting/enable-classic-confinement/example-classic-confinement-python.yaml
       :caption: snapcraft.yaml
       :language: yaml
       :lines: 31-34
       :dedent: 4

3. Override the part's build to also run a shell script that patches the ctype files:

   .. literalinclude:: ../code/crafting/enable-classic-confinement/example-classic-confinement-python.yaml
       :caption: snapcraft.yaml
       :language: yaml
       :lines: 26-28
       :dedent: 4

   The plugin now builds the part like normal before running the patch script.

4. Build the snap. During build, the script looks for core Python modules that need to
   be patched and refer to libraries in the base snap.


Go plugin
~~~~~~~~~

`golang-classic-example <https://github.com/snapcraft-docs/golang-classic-example>`_ is
a project with a main part that uses the Go plugin.

.. dropdown:: golang-classic-example project file

    .. literalinclude:: ../code/crafting/enable-classic-confinement/example-classic-confinement-go.yaml
        :language: yaml
        :caption: snapcraft.yaml
