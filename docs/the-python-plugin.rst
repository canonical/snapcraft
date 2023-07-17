.. 8529.md

.. _the-python-plugin:

The python plugin
=================

The ``python`` plugin can be used by either Python 2 or Python 3 based parts using a ``setup.py`` script for building the project, or using a package published to PyPI, and optionally any of the following:

-  a *requirements.txt* file used to import Python modules
-  packages installed directly from *pip*

This plugin uses the common plugin keywords as well as those for “sources”. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

Plugin-specific features and syntax are dependent on which :ref:`base <base-snaps>` is being used, as outlined below:

-  `base: core22 <the-python-plugin-core22_>`__
-  `base: core20 <the-python-plugin-core20_>`__
-  `base: core18 \| core <the-python-plugin-core18_>`__

See :ref:`Python applications <python-apps>` for a simple example, or search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+python%22&type=Code>`__ for projects already using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.


.. _the-python-plugin-core22:

base: core22
~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **python-requirements** (list) List of paths to requirements.txt file(s)
-  **python-constraints** (list) List of paths to constraint files.
-  **python-packages** (list, default: *[pip, setuptools, wheel]*) A list of dependencies to install using ``pip``. This supports the same syntax as the ``pip install`` command.

This plugin also interprets these specific build-environment entries:

-  ``PARTS_PYTHON_INTERPRETER`` (default: *python3*) The interpreter binary to search for in ``PATH``.

-  ``PARTS_PYTHON_VENV_ARGS`` Additional arguments for venv.

By default, this plugin uses Python from the base snap. If a part using this plugin uses a build-base other than that of the base, or a different interpreter is desired, it must be bundled in the snap (including *venv*) and must be in PATH.

It is required to bundle *python* when creating a snap that uses classic confinement, this can be accomplished on Ubuntu by adding stage-packages (i.e.; python3-venv).

Use of python3- in stage-packages will force the inclusion of the python interpreter.

Requires Snapcraft version *7.0+*.


.. _the-python-plugin-core20:

base: core20
~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **requirements** (array) List of paths to requirements.txt file(s)
-  **constraints** (string) Path to a constraints file.
-  **python-packages** (list) A list of dependencies to install using ``pip``. This supports the same syntax as the ``pip install`` command.

This plugin also interprets these specific build-environment entries:

-  ``SNAPCRAFT_PYTHON_INTERPRETER`` (default: *python3*) The interpreter binary to search for in ``PATH``.

-  ``SNAPCRAFT_PYTHON_VENV_ARGS`` Additional arguments for venv.

By default, this plugin uses Python from the base snap. If a part using this plugin uses a build-base other than that of the base, or a different interpreter is desired, it must be bundled in the snap (including *venv*) and must be in PATH.

It is required to bundle *python* when creating a snap that uses classic confinement, this can be accomplished on Ubuntu by adding stage-packages (i.e.; python3-venv).

Use of python3- in stage-packages will force the inclusion of the python interpreter.

Requires Snapcraft version *4.0+*.


.. _the-python-plugin-core18:

base: core18 \| core
~~~~~~~~~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **requirements** (array) List of paths to requirements.txt file(s)

-  **constraints** (string) Path to a constraints file

-  **process-dependency-links** (bool; default: false) Enable the processing of dependency links in pip, which allow one project to provide places to look for another project

-  **python-packages** (list) A list of dependencies to install using ``pip``. This supports the same syntax as the ``pip install`` command. For example:

   .. code:: yaml

      python-packages:
        - docopt == 0.6.1  # Install specific versions
        - git+https://github.com/inuits/mkdocs-factsheet.git  # Install from a git repository
        - https://github.com/cmacmackin/markdown-include/archive/v0.5.1.tar.gz  # Install from an archive

   See the `pip install docs <https://pip.pypa.io/en/stable/reference/pip_install/#pip-install>`__ for more information.

-  **python-version** (string; default: ``python3``) The python version to use. Valid options are ``python2`` and ``python3``

The ``python`` plugin also searches ``<stage-dir>/usr/bin/<python-interpreter>`` for a Python interpreter with a basename matching ``python-version`` in the ``<stage>`` directory. If detected, this takes preference and ``stage-packages`` will not use its own interpreter.
