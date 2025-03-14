# Starcraft

Welcome to Starcraft! We hope this document helps you get started. Before contributing
any code, please sign the
[Canonical Contributor Licence Agreement](http://www.ubuntu.com/legal/contributors).

## Setting up a development environment

We use a forking, feature-based workflow, so you should start by forking the repository.
Once you've done that, clone the project to your computer using the
`--recurse-submodules` parameter of `git clone`. See more in the [Git
submodules documentation](https://git-scm.com/book/en/v2/Git-Tools-Submodules#_cloning_submodules).

To set up a development environment, run:

```bash
make setup
```

To test that the environment works, run:

```bash
make lint
make test
```

### Code conventions

Several targets exist in the Makefile to help with code conventions. They are all merged
into the `format` target for automatic formatting and the `lint` target for running all
linters. These conventions apply to all files in the repository, including documentation
and metadata files. `pre-commit` is used for running many hooks before committing. Run:

```bash
make setup-precommit
```

to configure it for this repository.

### Testing

Tests can be run using `make test`, which will run all test forms. Specific types of
tests can be run with other testing targets shown in `make help`.

## Branches

Starcraft projects follow the
[OneFlow](https://www.endoflineblog.com/oneflow-a-git-branching-model-and-workflow) Git
branching model.

### Branch names

The branch name should be brief and less than 200 characters.

Branch names are formatted as `<category>/<name>`.

This naming convention provides a few benefits:

- GitHub workflows can choose which categories they should run on
- GitHub branches rules can be configured per category
- Some IDEs and Git tools display categorized branches in a neat and nested format

#### `main`

The main branch containing the latest changes.

#### `renovate/*`

Branches created automatically by the
[Renovate bot](https://github.com/renovatebot/renovate).

#### `work/<ticket>-<description>`

For any work driven by a ticketing system, the ticket should be part of the branch name.
The ticketing system can be built into the repo like GitHub issues or an external
ticketing system.

For GitHub issue #100 (a ticket to add a README file), the branch would be called
`work/100-add-readme`.

For an external ticketing system like Jira, a ticket labeled `CRAFT-100` would use a
branch called `work/CRAFT-100-add-readme`.

#### `work/<description>`

For any new features, bug fixes, and general work not driven by a ticketing system.

#### `hotfix/X.Y`

For hotfixes to an existing minor release.

For example, hotfixes for Testcraft 2.1 would go to a branch called `hotfix/2.1`.

As a departure from OneFlow, hotfix branches for *craft applications can be long-lived.
This is because *craft applications are built via Launchpad, which uses build recipes
that follow specific branches.

After a tagged release of a hotfix branch, the branch should be merged back to `main`.

#### `merge/<other-branch>`

For commits that merge another branch into the current branch. See the [chore](#chore)
section for information on merge commit headers.

#### `release/X.Y.Z`

For commits that prepare for a release. These commits should update the
[changelog](#changelog).

## Commits

Commit messages follow the [Conventional
Commits](https://www.conventionalcommits.org/en/v1.0.0/#summary) format:

> \<type>[(optional scope)][!]: \<description>
>
> [optional body]
>
> [optional footer]

The commit is divided into three sections: a header, body, and footer.

### Header

The header is required and consists of three subsections: a type, optional scope, and
description. The header must be 72 characters or less.

#### Types

##### `ci`

Commits that affect the CI/CD pipeline.

##### `build`

Commits that affect the build of an application or library.

This includes dependency updates, which should use the `deps` scope (`build(deps):`).

##### `feat`

Commits that add a new feature for the user.

##### `fix`

Commits that fix a bug or regression.

##### `perf`

Commits that improve performance without changing the API or external behavior.

##### `refactor`

Commits that refactor code.

Using [Martin Fowler's definition](https://refactoring.com/), refactor means

> a change made to the internal structure of software to make it easier to understand
> and cheaper to modify without changing its observable behavior.

##### `style`

Commits that change the syntax, format, or aesthetics of any text in the codebase. The
meaning of the text should not change.

Examples include:

- Automatic changes from tools like `black` and `ruff format`
- Changes to documentation that don't affect the meaning
- Correcting a typo

##### `test`

Commits that improve, add, or remove tests.

##### `docs`

Commits that affect the contents of the documentation.

Changes to how documentation is built should use `build(docs):`.

Changes to how the documentation is built in the CI/CD pipeline should use the
`ci(docs):`.

##### `chore`

Miscellaneous commits that don't fit into any other type.

Examples include:

- Edits to a comment or docstring
- Type changes
- Accommodating a developer-facing deprecation warning
- Many _small_ fixes for an existing PR
- Reverts (`chore(revert): <header of reverted commit>`)
- Merge commits (`chore(merge): <branch or tag> into <branch>`)
    - The remote name should not be included (for example, use `main` instead of
      `origin/main`)

##### Choosing the right type

Sometimes, multiple types may be appropriate for a PR.

This may signal that a commit is doing more than one thing and should be broken into
multiple smaller commits. For example, a commit should not refactor code and fix a bug.
This should be two separate commits.

In other scenarios, multiple types could be appropriate because of the nature of the
commit. This can happen with `test` and `docs`, which can be used as types or scopes.

The types above are ordered by descending priority. The first appropriate type should be
used.

For example, refactoring a test suite could have the header
`test(project): reorganize tests` or `refactor(test): reorganize project tests`.
`refactor` has a higher priority than `test`, so the latter option is correct.

#### Scope

A scope is an optional part of the commit header. It adds additional context by
specifying what part of the codebase will be affected.

It should be a tangible part of the codebase, like a directory, module, or class name.

If a commit affects many areas of the codebase, the scope should be omitted; `many` is
not an accepted scope.

#### Breaking changes

If an exclamation point (!) is inserted after the type/scope, this means that the
commit introduces a breaking change. Including one or more commits with an exclamation
point in a release will trigger a major version increment.

Breaking changes may also be indicated by including the words `BREAKING CHANGE` in the
commit footer.

#### Description

The description is written in the imperative mood (present tense, second person). The
description should complete the following sentence:

> If applied, this commit will \<description>.

The description does not begin with capital letter (unless it's a proper noun) and does
not end with puncuation mark.

#### Example commit headers

> feat: inherit context from services

> test: increase unit test stability

> fix: check foo before running bar

> feat(daemon): foo the bar correctly in the baz

> test(daemon): ensure the foo bars correctly in the baz

> fix(test): mock class Foo

> ci(snap): upload the snap artefacts to Github

> chore(deps): update go.mod dependencies

### Body

The body is an optional section of the commit to provide more context. It should be
succinct (no more than 3-4 sentences) and may reference relevant bugs and issues.

### Footer

The footer is an optional section of the commit message that can mention the signer and
co-authors of the commit.

Example footers:

> Signed-off-by: <name> <<email>>

> Co-authored-by: <name> <<email>>

## Changelog

### Scope

The changelog is a reference documentation page that gives a human-readable summary of
changes to the project that are relevant to users.

Each change should be clear in its purpose, whether it is fixing a bug, adding a
feature, or changing an existing behavior.

Internal changes should not be included in the changelog. For example, dev dependency
updates, CI updates, and style changes should not be included.

### Style and format

Changes should be written in the imperative mood (present tense, second person) similar
to commit headers.

The changelog should link to the project's GitHub releases page, which contains an
exhaustive list of all commits added to the release.

Release entries should be sorted by date from newest to oldest.

### Hotfixes

If an older version gets a hotfix release, subsequent releases should mention when the
hotfix is incorporated.

For example, consider a package with a previous release `2.9.0` and a latest release of
`3.0.0`. If the `2.9.0` receives a hotfix release `2.9.1` and is merged back to `main`,
then the next `3.x` release should indicate that it includes the changes from 2.9.1, for
example:

```rst
.. note::

    3.0.1 includes changes from the 2.9.1 release.
```
