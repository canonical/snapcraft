.. _reference-conda-plugin:

Conda plugin
============

The Conda plugin builds parts by retrieving packages with the `conda
<https://docs.conda.io/>`_ package manager. Though conda is language-agnostic, the
plugin is most commonly used to incorporate Python packages into snaps.


Keywords
--------

The plugin provides the following unique keys for core20 and newer snaps.


conda-packages
~~~~~~~~~~~~~~
**Type**: list of strings

**Default**: ``unset``

The conda packages to install.


conda-python-version
~~~~~~~~~~~~~~~~~~~~
**Type**: string

**Default**: ``unset``

The Python version to install in the `conda environment
<https://docs.conda.io/projects/conda/en/latest/user-guide/concepts/environments.html>`_,
specified with both a major and minor version (for example, 3.12). If unset, Python
isn't installed in the environment.


conda-miniconda-version
~~~~~~~~~~~~~~~~~~~~~~~
**Type**: string

**Default**: ``latest``

The version of `Miniconda <https://docs.conda.io/en/latest/miniconda.html>`_ to
bootstrap. If unset, the latest release is bootstrapped.


Dependencies
------------

This plugin installs ``curl`` for use during the Miniconda installation.


How it works
------------

#. Download the Miniconda installation script with ``curl``, respecting the version
   declared with ``conda-miniconda-version``.
#. Install Miniconda in the part's build environment with the ``miniconda.sh`` script.
#. Create a conda environment in the part's install directory with ``conda create``. In
   this environment, conda installs any packages declared with ``conda-packages`` and
   the Python version declared with ``conda-python-version``.


Example
-------

The test suite in Snapcraft has a `conda-hello
<https://github.com/canonical/snapcraft/blob/main/tests/spread/plugins/v2/snaps/conda-hello/snap/snapcraft.yaml>`_
snap built with the Conda plugin.
