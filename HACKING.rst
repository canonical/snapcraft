*********
Starcraft
*********

Welcome to Starcraft! We hope this document helps you get started. Before
contributing any code, please sign the `Canonical contributor licence
agreement`_.

Setting up a development environment
------------------------------------
We use a forking, feature-based workflow, so you should start by forking the
repository. Once you've done that, clone the project to your computer using git
clone's ``--recurse-submodules`` parameter. (See more on the `git submodules`_
documentation.)

Tooling
=======
We use a large number of tools for our project. Most of these are installed for
you with tox, but you'll need to install:

- Python 3.8 (default on Ubuntu 20.04, available on Ubuntu 22.04 through the
  deadsnakes_ PPA) with setuptools.
- tox_ version 3.8 or later
- ShellCheck_  (also available via snap: ``snap install shellcheck``)

Once you have all of those installed, you can install the necessary virtual
environments for this repository using tox.

Other tools
###########
Some other tools we use for code quality include:

- Black_ for code formatting
- pytest_ for testing
- ruff_ for linting (and some additional formatting)

A complete list is kept in our pyproject.toml_ file in dev dependencies.

Initial Setup
#############

After cloning the repository but before making any changes, it's worth ensuring
that the tests, linting and tools all run on your machine. Running ``tox`` with
no parameters will create the necessary virtual environments for linting and
testing and run those::

    tox

If you want to install the environments but not run the tests, you can run::

    tox --notest

If you'd like to run the tests with a newer version of Python, you can pass a
specific environment. You must have an appropriately versioned Python
interpreter installed. For example, to run with Python 3.10, run::

    tox -e test-py3.10

While the use of pre-commit_ is optional, it is highly encouraged, as it runs
automatic fixes for files when ``git commit`` is called, including code
formatting with ``black`` and ``ruff``.  The versions available in ``apt`` from
Debian 11 (bullseye), Ubuntu 22.04 (jammy) and newer are sufficient, but you can
also install the latest with ``pip install pre-commit``. Once you've installed
it, run ``pre-commit install`` in this git repository to install the pre-commit
hooks.

Tox environments and labels
###########################

We group tox environments with the following labels:

* ``format``: Runs all code formatters with auto-fixing
* ``type``: Runs all type checkers
* ``lint``: Runs all linters (including type checkers)
* ``unit-tests``: Runs unit tests in several supported Python versions
* ``integration-tests``: Run integration tests in several Python versions
* ``tests``: The union of ``unit-tests`` and ``integration-tests``

For each of these, you can see which environments will be run with ``tox list``.
For example::

    tox list -m lint

You can also see all the environments by simply running ``tox list``

Running ``tox run -m format`` and ``tox run -m lint`` before committing code is
recommended.

.. _Black: https://black.readthedocs.io
.. _`Canonical contributor licence agreement`: http://www.ubuntu.com/legal/contributors/
.. _deadsnakes: https://launchpad.net/~deadsnakes/+archive/ubuntu/ppa
.. _`git submodules`: https://git-scm.com/book/en/v2/Git-Tools-Submodules#_cloning_submodules
.. _pre-commit: https://pre-commit.com/
.. _pyproject.toml: ./pyproject.toml
.. _Pyright: https://github.com/microsoft/pyright
.. _pytest: https://pytest.org
.. _ruff: https://github.com/charliermarsh/ruff
.. _ShellCheck: https://www.shellcheck.net/
.. _tox: https://tox.wiki
