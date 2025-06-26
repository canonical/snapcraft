# Contributing

Snapcraft has a community from all over the world, and we welcome all contributions.

All contributors should become familiar with this guide. It outlines the expectations
and practices for participating in the project.

## Review the project expectations

The Starcraft team at Canonical sets the direction and priorities of Snapcraft. They
take responsibility for its stewardship and health.

Before you start work on Snapcraft, there are three documents for you to digest.

### Ubuntu Code of Conduct

Projects governed by Canonical expect good conduct and excellence from every member. The
specific principles are laid out in the [Ubuntu Code of
Conduct](https://ubuntu.com/community/ethos/code-of-conduct).

### Canonical Contributor License Agreement

As a contributor, you retain your copyright and attribution rights, provided you sign
the [Canonical Contributor License Agreement](http://www.ubuntu.com/legal/contributors).
Before committing any code, review its terms. If you agree and sign it, your code can be
incorporated into the repository.

### Open source license

Snapcraft is licensed under [GPL-3.0](LICENSE).

## Report an issue or open a request

If you find a bug or feature gap in Snapcraft, look for it in the [project's GitHub
issues](https://github.com/canonical/snapcraft/issues) first. If you have fresh input,
add your voice to the issue.

If the bug or feature doesn't have an issue, we invite you to [open
one](https://github.com/canonical/snapcraft/issues/new/choose).

## Set up for development

Snapcraft uses a forking, feature-based workflow. Most work on Snapcraft occurs on
people's local systems, and is heavily terminal-dependent. Remote testing and building
is provided on GitHub for continuous integration and delivery.

Start by [creating a personal fork](https://github.com/canonical/snapcraft/fork) of the
repository on GitHub.

Next, on your host system, clone your fork:

```bash
git clone git@github.com:<username>/snapcraft.git --recurse-submodules
```

Inside the project directory, set up the virtual development environment and install all
dependencies, linters, and testers:

```bash
make setup
make lint
make test
```

If all linting and testing completes without errors, your local environment is ready.

## Contribute a change

With the prerequisites out of the way, let's walk through how to make a contribution to
Snapcraft.

### Research the topic

All significant work in Snapcraft should be tied to an existing issue or ticket.

If you'd like to add a small feature or fix, check the project's GitHub issues to see if
others have reported it. If they have, look into the current status of the topic. If no
one else is working on it, add a comment stating that you'd like to take it on, and the
Starcraft team will assign it to you.

If there's a large feature or fix you'd like to work on, contact the team and the
community on the [Snapcraft Matrix channel](https://matrix.to/#/#snapcraft:ubuntu.com).
It's possible that work on it has been started, or that it fits into an existing plan.
Often, you will save time and effort by checking for prior work.

If you're ever in doubt about developments in the project, ask!

### Apply for a bounty

Snapcraft has a parallel stream of materially rewarding work in the form of bounties. At
the Starcraft team's discretion, high-value GitHub issues are allocated monetary
bounties.

If you're interested in bounties, enroll in the [GitHub Sponsors
program](https://docs.github.com/en/sponsors/getting-started-with-github-sponsors/about-github-sponsors).

To show interest in a particular bounty, submit a high-level solution plan as a comment
on its issue. The Snapcraft maintainers will review it for technical soundness. If your
proposal is accepted, they will assign you to the bounty pool, after which you can begin
work.

### Create a development branch

Once you've settled on a topic to work on, it's time to set up a local branch.

Always start by syncing against the branch and the dependencies you're basing your
changes on.

If you're making a change to a current release, run:

```bash
git checkout hotfix/<current-release>
git pull
make setup
```

If you're contributing to multiple releases or the next major release, run:

```bash
git checkout main
git pull
make setup
```

Next, create a new branch against your chosen base. The new branch name should be brief,
at no more than 80 characters.

If you're working on a ticket, format your branch name as `<ticket-id>-<description>`.
For example, if you're working on GitHub issue \#235, and it's about adding a string
sanitizer, use the format:

```bash
git checkout -b issue-235-add-string-sanitizer-method
```

If you have a small ad-hoc change with no ticket, make the name distinct and meaningful.
For example, if you're fixing a typo, use the format:

```bash
git checkout -b string-sanitizer-fix-typo
```

### Commit a change

Once you've made the changes to the code and you're ready to test it, start by
committing:

```bash
git add -A
git commit
```

Format the commit message according to the [Conventional
Commits](https://www.conventionalcommits.org/en/v1.0.0/) style. For the sanitizer
example, an appropriate commit title would be:

```
feat: add text sanitizer
```

If you need help determining the type of conventional commit for your change, look at
the history of the file in question with `git log --oneline <filename>` . When you're
done browsing, press `Q` to exit the interactive log.

> **Tip**
>
> With complex changes, you might get stuck choosing a conventional commit type.
>
> This may signal that a commit is doing more than one thing and should be broken into
> multiple smaller commits. A commit should not, for example, refactor code and fix a
> bug. That should be two separate commits.
>
> In other scenarios, multiple types could be appropriate because of the nature of the
> commit. This can happen with test and docs, which can be used as either types or
> scopes.
>
> Run down the following list and select the highest-ranked type that fits your change:
>
> - ci
> - build
> - feat
> - fix
> - perf
> - refactor
> - style
> - test
> - docs
> - chore

Committing triggers the pre-commit hook, which runs the automatic code formatter and the
fast linters.

If the linters reformatted any of the files, the commit was cancelled. To make the
changes stick, restage the modified files with `git add -A` and commit again.

### Test the change

Test early and often, especially before you plan to open a pull request.

For low-complexity changes that require basic unit testing, run the fast tests:

```bash
make test-fast
```

For complex work, run the full test suite:

```bash
make test
```

Running all tests can take a very long time, in some cases an hour.

When iterating and testing, it's a good practice to clean the local temporary files that
the tests generate:

```bash
make clean
```

In rare instances, tests can fail in unpredictable ways, regardless of the state of your
code. In such cases, it's best to delete your virtual environment and start over:

```bash
rm -rf ./.venv
make clean
make setup
```

There are also special tests that are feature-specific. Run `make help` to view all of
them.

### Document the change

Snapcraft has frequent documentation updates to both its user guide and its developer
material.

Document feature changes and fixes in the relevant [release notes](docs/release-notes/).

For all documentation updates, build the site locally with:

```bash
make docs
```

Use the special documentation linter to check for errors:

```bash
make lint-docs
```

### Push the branch and open a PR

Once your work is committed to your branch, push it to your fork:

```bash
git push -u origin <branch-name>
```

Finally, [open a PR](https://github.com/canonical/snapcraft/compare) for it on GitHub.
If your branch has one commit, GitHub will title the PR after it. If your branch has
more than one commit, name the PR after the most significant. Once open, reviewers are
assigned automatically to your work.

### Follow up for the review

When a maintainer reviews your PR, they typically leave inline comments and suggestions
on the code.

If the comment is a request, accommodate it by pushing one or more additional commits to
the branch. It's simplest to add the commits locally and push, rather than in the GitHub
interface, as it leads to fewer potential conflicts with syncs.

Don't force-push changes to the branch. It destroys the history of the review and makes
it harder for maintainers to see code changes.

### Evaluating pull requests

Oftentimes all you want to do is see if a given pull request solves the issue you were
having. To make this easier, snaps are published for AMD64 and ARM64 on a channel named
`latest/edge/pr-<PR-number>` where `PR number` is the number of the pull request.

For feature branches, snaps are published for AMD64 and ARM64 on a channel named
`latest/edge/<branch-name>`. For example, a branch named `feature/offline-mode` would be
available on the channel `latest/edge/offline-mode`.
