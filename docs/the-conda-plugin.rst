.. 12530.md

.. _the-conda-plugin:

The conda plugin
================

The ``conda`` plugin is useful, primarily, for Python parts using the `Conda <https://docs.conda.io>`__ open source package management system.

This plugin uses the common plugin keywords as well as those for “sources”. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

Additional features and syntax are dependent on which :ref:`base <base-snaps>` is being used, as outlined below:

-  `base: core22 <the-conda-plugin-core22_>`__
-  `base: core20 <the-conda-plugin-core20_>`__
-  `base: core18 \| core <the-conda-plugin-core18_>`__

For examples, search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+conda%22&type=Code>`__ for projects already using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.


.. _the-conda-plugin-core22:

base: core22
~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

* **conda-packages** (list of strings) List of *conda* packages to install.
* **conda-python-version** (string) The Python version to use for the *conda* packages. Python version major and minor version (e.g. 3.9).
* **conda-miniconda-version** (string, default: *latest*) The version of `Miniconda <https://docs.conda.io/en/latest/miniconda.html>`__ to bootstrap. Defaults to the latest release.

Requires Snapcraft version *7.0+* .


.. _the-conda-plugin-core20:

base: core20
~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

* **conda-packages** (list of strings) List of *conda* packages to install.
* **conda-python-version** (string) The Python version to use for the *conda* packages. Python version major and minor version (e.g. 3.9).
* **conda-miniconda-version** (string, default: *latest*) The version of `Miniconda <https://docs.conda.io/en/latest/miniconda.html>`__ to bootstrap. Defaults to the latest release.

Requires Snapcraft version *4.6+* .


.. _the-conda-plugin-core18:

base: core18 \| core
~~~~~~~~~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

- **conda-packages** (list of strings) List of *conda* packages to install.
- **conda-python-version** (string) The Python version to use for the *conda* packages. Defaults to the latest supported by `Miniconda <https://docs.conda.io/en/latest/miniconda.html>`__.
- **conda-miniconda-version** (string) The version of `Miniconda <https://docs.conda.io/en/latest/miniconda.html>`__ to bootstrap. Defaults to the latest release.
