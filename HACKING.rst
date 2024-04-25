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
- Codespell_ (also available via snap: ``snap install codespell``)
- ruff_ (also available via snap: ``snap install ruff``)

Once you have all of those installed, you can install the necessary virtual
environments for this repository using tox.

Other tools
###########
Some other tools we use for code quality include:

- Black_ for code formatting
- pytest_ for testing

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

Commits
-------

Commit messages are based on the `conventional commit`_ style::

  <type>(<optional scope>): <description>

  <optional body>

  <optional footer>

The commit is divided into three sections: a header, body, and footer.

Header
======

The header is required and consists of three subsections: a type,
optional scope, and description. The header must be 72 characters or less.

Types
#####

``ci``
""""""

Commits that affect the CI/CD pipeline.

``build``
"""""""""

Commits that affect the build of an application or library.

This includes dependency updates, which should use the ``deps`` scope
(``build(deps):``).

``feat``
""""""""

Commits that add a new feature for the user.

``fix``
"""""""

Commits that fix a bug or regression.

``perf``
""""""""

Commits that improve performance without changing the API or external behavior.

``refactor``
"""""""""""""

Commits that refactor code.

Using `Martin Fowler's definition`_, refactor means "*a change made
to the internal structure of software to make it easier to understand and
cheaper to modify without changing its observable behavior.*"

``style``
""""""""""

Commits that change the syntax, format, or aesthics of any text the codebase.
The meaning of the text should not change.

Examples include:
* automatic changes from tools like ``black`` and ``ruff format``
* changes to documentation that don't affect the meaning
* correcting a typo

``test``
""""""""

Commits that improve, add, or remove tests.

``docs``
""""""""

Commits that affect the contents of the documentation.

Changes to how documentation is built should use ``build(docs)::``.

Changes to how the documentation is built in the CI/CD pipeline should use
the ``ci(docs):``.

``chore``
"""""""""

Miscellaneous commits that don't fit into any other type.

Examples include:

* edits to a comment or docstring
* type changes
* accommodating a developer-facing deprecation warning
* many *small* fixes for an existing PR
* merge commits (``chore(merge): '<branch1>' into '<branch2>'``)

  * the remote name should not be included (i.e. use ``'main'``
    instead of ``'origin/main'``)

Choosing the right type
"""""""""""""""""""""""

Sometimes, multiple types may be appropriate for a PR.

This may signal that a commit is doing more than one thing and should be
broken into multiple smaller commits. For example, a commit should not refactor
code and fix a bug. This should be two separate commits.

In other scenarios, multiple types could be appropriate because of the nature
of the commit. This can happen with ``test`` and ``docs``, which can be used
as types or scopes.

The types above are ordered by descending priority. The first appropriate type
should be used.

For example, refactoring a test suite could have the header
``test(project): reorganize tests`` or
``refactor(test): reorganize project tests``. ``refactor`` has a higher
priority than ``test``, so the latter option is correct.


Scope
#####

A scope is an optional part of the commit header.  It adds additional context
by specifying what part of the codebase will be affected.

It should be a tangible part of the codebase, like a directory, module, or
class name.

If a commit affects many areas of the codebase, the scope should be omitted;
``many`` is not an accepted scope.

Description
###########

The description is written in the imperative mood (present tense, second
person). The description should complete the following sentence::

  If applied, this commit will <description>.

The description does not begin with capital letter (unless it's a proper
noun) and does not end with puncuation mark.

Examples
########

Examples of commit headings::

    feat: inherit context from services
    test: increase unit test stability
    fix: check foo before running bar
    feat(daemon): foo the bar correctly in the baz
    test(daemon): ensure the foo bars correctly in the baz
    fix(test): mock class Foo
    ci(snap): upload the snap artefacts to Github
    chore(deps): update go.mod dependencies

Body
====

The body is an optional section of the commit to provide more context.
It should be succinct (no more than 3-4 sentences) and may reference relevant
bugs and issues.

Footer
======

The footer is an optional section of the commit message that can mention the
signer and co-authors of the commit.

Example footers::

  Signed-off-by: <name> <<email>>
  Co-authored-by: <name> <<email>>


.. _Black: https://black.readthedocs.io
.. _`Canonical contributor licence agreement`: http://www.ubuntu.com/legal/contributors/
.. _Codespell: https://github.com/codespell-project/codespell
.. _`conventional commit`: https://www.conventionalcommits.org/en/v1.0.0/#summary
.. _deadsnakes: https://launchpad.net/~deadsnakes/+archive/ubuntu/ppa
.. _`git submodules`: https://git-scm.com/book/en/v2/Git-Tools-Submodules#_cloning_submodules
.. _`Martin Fowler's definition`: https://refactoring.com/
.. _pre-commit: https://pre-commit.com/
.. _pyproject.toml: ./pyproject.toml
.. _Pyright: https://github.com/microsoft/pyright
.. _pytest: https://pytest.org
.. _ruff: https://github.com/charliermarsh/ruff
.. _ShellCheck: https://www.shellcheck.net/
.. _tox: https://tox.wiki
