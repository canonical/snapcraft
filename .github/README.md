[![Release](https://github.com/canonical/starbase/actions/workflows/release-publish.yaml/badge.svg?branch=main&event=push)](https://github.com/canonical/starbase/actions/workflows/release-publish.yaml)
[![Documentation](https://github.com/canonical/starbase/actions/workflows/docs.yaml/badge.svg?branch=main&event=push)](https://github.com/canonical/starbase/actions/workflows/docs.yaml)
[![test](https://github.com/canonical/starbase/actions/workflows/tests.yaml/badge.svg?branch=main&event=push)](https://github.com/canonical/starbase/actions/workflows/tests.yaml)

# starbase

The base repository for Starcraft projects.

## Description

This template code is the basis for all future starcraft projects, and acts as the
testbed for any major tooling changes that we want to make before propagating them
across all projects.

## Structure

TODO

## Migrate existing projects

1. Update this guide as you go along, if something is unclear or missing.
2. Use ruff.
    1. Pull in the bare minimum `pyproject.toml` needed to use ruff.

    2. Make your codebase pass with ruff. Commit after each step:
        1. `ruff check --fix`
        2. `ruff check --fix --unsafe-fixes`
        3. `ruff check --add-noqa`
        4. `ruff format`

    3. Replace use of black, flake8, pydocstyle, isort, and pylint in Makefile/CI with:
        - `ruff check --fix`
        - `ruff format`

3. Modify top-level files in your project to match what's in Starbase as closely as
   possible.
    1. `Makefile` – Ensure you use `uv` and at least have the same targets:
        - `setup`
        - `lint`
        - `test-unit`
        - `test-integration` (If this applies to your repo, i.e. the repo is a library
          rather than an application)
        - `coverage`

    2. `pyproject.toml` – Expand from just the ruff things: move things into here from
       your `setup.py`, `setup.cfg`, and `requirements.*.txt`.

    3. Project files – If any of the repository docs (README, CONTRIBUTING, and so on)
       aren't written in Markdown, convert them to `.md` with pandoc:

        `pandoc -o README.md README.rst`

        Be sure to review the changes it makes. Pandoc sometimes does unexpected things
        when interpreting reStructuredText directives.

    4. `README.md` – Base the contents of the README on the provided [README
       template](README.md). Try and align with the model as closely as you can. For an
       example of this template in use, refer to the [Snapcraft
       README](https://github.com/canonical/snapcraft/blob/6fa334402a1ebaa6ff58064d49f1f46fd01fb404/README.md).

    5. `SECURITY.md` – Match the content of your project's `SECURITY.md` file as closely
       as you can to the structure of the [Security policy template](SECURITY.md). Be
       sure to follow the comments in the template closely.

4. Run all the linters with `make lint`.
    1. `mypy`:
        - Mypy checks the same things as `ruff`'s `ANNXXX` checks, but `ruff`'s `noqa`
          directives mean nothing to mypy. You'll need to fix these by hand, mostly by
          adding type annotations to function definitions.

    2. `pyright`:
        - For errors along the lines of "Stub file not found for $library", check for
          the existence of pip package `typing-$library` and add it as a dependency.
        - If you have lots of errors you may need to remove the
          `strict` directive from `pyproject.toml`.

5. Do a side-by-side diff of the `.gitignore` files in your project and Starbase, making
   them as close as possible and adding anything that makes sense upstream.

6. Bring in remaining top-level files:
    - `.editorconfig`
    - `.pre-commit-config.yaml`
    - `.shellcheckrc`
    - `tox.ini`

7. If you're rebasing a library, add the integrations tests structure. Applications
   should use spread for integration tests.

8. Finally, once all files are manually synced, actually sync the Git history:
    - `git remote add starbase git@github.com:canonical/starbase.git`
    - `git fetch starbase main`
    - `git merge --allow-unrelated-histories starbase/main`
    - `git remote remove starbase`
    - Don't forget to review all the new files and dirs that this merge adds -you'll
      want to delete a lot of them.
    - When you merge, DO NOT squash, otherwise the starbase history will not be
      preserved.

## Create a new project

1. [Use this
   template](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template)
   to create your repository.

2. Sync the Git history with starbase to ease future merging:
    - `git clone <your-repo>`
    - `git remote add starbase git@github.com:canonical/starbase.git`
    - `git fetch starbase main`
    - `git merge --allow-unrelated-histories starbase/main`
    - `git push -f origin main`
    - `git remote remove starbase`

3. Ensure the `LICENSE` file represents the current best practices from the Canonical
   legal team for the specific project you intend to release. We use LGPL v3 for
   libraries, and GPL v3 for apps.

4. Rename any files or directories and ensure references are updated.

5. Replace any instances of the words `Starcraft` and `starbase` with the product's
   name.

6. Place contact information in a code of conduct.

7. Rewrite the README.

8. If a Diataxis quadrant (tutorials, how-tos, references, explanations) doesn't yet
   have content, remove its landing page from the TOC and delete its card in
   `docs/index.rst`. You can re-index it when at least one document has been produced
   for it.

9. Register the product's documentation on our custom domain on [Read the Docs for
   Business](https://library.canonical.com/documentation/publish-on-read-the-docs)

10. Delete `.github/README.md`.
