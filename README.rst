|Release| |Documentation| |test|

.. |Release| image:: https://github.com/canonical/starbase/actions/workflows/release-publish.yaml/badge.svg?branch=main&event=push
   :target: https://github.com/canonical/starbase/actions/workflows/release-publish.yaml
.. |Documentation| image:: https://github.com/canonical/starbase/actions/workflows/docs.yaml/badge.svg?branch=main&event=push
   :target: https://github.com/canonical/starbase/actions/workflows/docs.yaml
.. |test| image:: https://github.com/canonical/starbase/actions/workflows/tests.yaml/badge.svg?branch=main&event=push
   :target: https://github.com/canonical/starbase/actions/workflows/tests.yaml

********
starbase
********

The base repository for Starcraft projects.

Description
-----------
This template code is the basis for all future starcraft projects, and acts as
the testbed for any major tooling changes that we want to make before
propagating them across all projects.

Structure
---------
TODO

Migrate existing projects
--------------------------------
#. Update this guide as you go along, if something is unclear or missing.

#. Use ruff.
   #. Pull in the bare minimum ``pyproject.toml`` needed to use ruff.
   #. Make your codebase pass with ruff.  Commit after each step:

      #. ``ruff check --fix``
      #. ``ruff check --fix --unsafe-fixes``
      #. ``ruff check --add-noqa``
      #. ``ruff format``

   #. Replace use of black, flake8, pydocstyle, isort, and pylint in Makefile/CI
      with:
      - ``ruff check --fix``
      - ``ruff format``
#. Modify top-level files in your project to match what's in Starbase as closely
   as possible.
   #. ``Makefile`` - Ensure you use ``uv`` and at least have the same targets:

      - ``setup``
      - ``lint``
      - ``test-unit``
      - ``test-integration`` (If this applies to your repo, i.e. the repo is a library
        rather than an application)
      - ``coverage``

   #. ``pyproject.toml`` - Expand from just the ruff things: move things into
      here from your ``setup.py``, ``setup.cfg``, and ``requirements.*.txt``.
   #. ``README`` - If your readme is .md, convert to .rst with pandoc:
      ``pandoc -o README.rst README.md``
      Don't worry about making the contents match, Starbase's is very specific.
#. Run all the linters: ``make lint``
   #. ``mypy``:

      - Mypy checks the same things as ``ruff``'s ``ANNXXX`` checks, but
        ``ruff``'s ``noqa`` directives mean nothing to mypy.  You'll need to fix
        these by hand, mostly by adding type annotations to function definitions.

   #. ``pyright``:

      - For errors along the lines of "Stub file not found for $library", check
        for the existence of pip package ``typing-$library`` and add it as a
        dependency.
      - If you have lots of errors you may need to remove the ``strict``
        directive from ``pyproject.toml``.

#. Do a side-by-side diff of the ``.gitignore`` files in your project and
   Starbase, making them as close as possible and adding anything that makes
   sense upstream.

#. Bring in remaining top-level files:
   - .editorconfig
   - .pre-commit-config.yaml
   - .shellcheckrc
   - tox.ini
   - .yamllint.yaml

#. If you're rebasing a library, add the integrations tests structure.
   Applications should use spread for integration tests.

# Finally, once all files are manually synced, actually sync the git history:
   - ``git remote add starbase git@github.com:canonical/starbase.git``
   - ``git merge --allow-unrelated-histories starbase/main``
   - ``git remote remove starbase``
   - Don't forget to review all the new files and dirs that this merge adds -
     you'll want to delete a lot of them.
   - When you merge, DO NOT squash, otherwise the starbase history will not be
     preserved.


Create a new project
--------------------

#. `Use this template`_ to create your repository.
#. Ensure the ``LICENSE`` file represents the current best practices from the
   Canonical legal team for the specific project you intend to release. We use
   LGPL v3 for libraries, and GPL v3 for apps.
#. Rename any files or directories and ensure references are updated.
#. Replace any instances of the word ``Starcraft`` with the product's name.
#. Place contact information in a code of conduct.
#. Rewrite the README.
#. If a Diataxis quadrant (tutorials, how-tos, references, explanations)
   doesn't yet have content, remove its landing page from the TOC and delete
   its card in ``docs/index.rst``. You can re-index it when at least one
   document has been produced for it.
#. Register the product's documentation on our custom domain on `Read the
   Docs for Business`_.

.. _EditorConfig: https://editorconfig.org/
.. _pre-commit: https://pre-commit.com/
.. _Read the Docs for Business: https://library.canonical.com/documentation/publish-on-read-the-docs
.. _use this template: https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template
