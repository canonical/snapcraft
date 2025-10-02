# Contributing

Starbase has a community from all over the world, and the Starbase team welcomes all
contributions.

Contributing offers an opportunity to polish your technical skills, develop as a
professional, and get involved in the growing open source community. Starbase
contributors are also recognized in any releases that they work on.

All contributors should become familiar with this guide. It outlines the expectations
and practices for participating in the project.

## Review the project expectations

The Starcraft team at Canonical sets the direction and priorities of Starbase. They
take responsibility for its stewardship and health.

Before you start work on Starbase, there are three documents for you to digest.

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

<Ensure that the link text matches your project's license.>

Starbase is licensed under [LGPL-3.0](LICENSE).

## Report an issue or open a request

If you find a bug or feature gap in Starbase, look for it in the [project's GitHub
issues](https://github.com/canonical/starbase/issues) first. If you have fresh input,
add your voice to the issue.

If the bug or feature doesn't have an issue, we invite you to [open
one](https://github.com/canonical/starbase/issues/new/choose).

<Cut the following section if your project doesn't have bounties.>

## Apply for a bounty

Starbase has a parallel stream of materially-rewarding work in the form of bounties. At
the Starcraft team's discretion, high-value GitHub issues are allocated monetary
bounties. A bounty is paid when the solution is merged into the codebase and its
implementation meets all business and technical specifications outlined in the issue. To
keep things fair, you can work on only one bounty at a time.

If you're interested in bounties, enroll in the [GitHub Sponsors
program](https://docs.github.com/en/sponsors/getting-started-with-github-sponsors/about-github-sponsors).

To show interest in a bounty, provide a rough solution plan for it in a comment on its
GitHub issue. The Starbase maintainers review all proposals for technical soundness. If
your proposal is accepted, they will assign you the bounty, after which you can begin
work.

## Set up for development

Starbase uses a forking, feature-based workflow. Most work on Starbase occurs on
people's local systems, and is heavily terminal-dependent. Remote testing and building
is provided on GitHub for continuous integration and delivery.

Start by [creating a personal fork](https://github.com/canonical/starbase/fork) of the
repository on GitHub.

Next, on your host system, clone your fork and sync it with the upstream repository.

If you authenticate your GitHub account with
[SSH](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account),
run:

```bash
git clone git@github.com:<username>/starbase --recurse-submodules
cd starbase
git remote add upstream git@github.com:canonical/starbase
git fetch upstream
```

If you don't authenticate with SSH, clone with
[HTTPS](https://docs.github.com/en/get-started/git-basics/about-remote-repositories#cloning-with-https-urls)
instead:

```bash
git clone https://github.com/<username>/starbase --recurse-submodules
cd starbase
git remote add upstream https://github.com/canonical/starbase
git fetch upstream
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
Starbase.

### Research the topic

All significant work in Starbase should be tied to an existing issue or ticket.

Once you find an issue that you'd like to work on, it's best to reach out so that a
Starbase maintainer can assign the ticket to you before starting. This process varies
depending on the issue's complexity and scope.

#### Minor changes

If you'd like to add a small feature or fix, check the project's GitHub issues to see if
others have reported it. If they have, look into the current status of the topic. If no
one else is working on it, add a comment stating that you'd like to take it on, and a
Starbase maintainer will assign it to you.

If you don't find a related issue, [open
one](https://github.com/canonical/starbase/issues/new/choose) and indicate that you're
interested in taking it on. When creating the issue, be sure to add any relevant labels.
A Starbase maintainer will then review the issue and assign it to you.

#### Major changes

<Update the link to point to your project's Matrix channel.>

If there's a large feature or fix you'd like to work on, contact us and the rest of the
community in the [Starcraft Development Matrix
space](https://matrix.to/#/#starcraft-development:ubuntu.com). It's possible that work
on it has been started, or that it fits into an existing plan. Often, you will save time
and effort by checking for prior work.

<If your project has a forum, link it here. If not, remove this paragraph.>

For changes that require coordination with the the broader community, create a post on
the [Starbase forum](https://forum.example.com), the hub for users and developers.

Once you've found or created an issue you'd like to take on, propose your solution in
the issue's thread. In your proposal, describe a plan for the change, its tests, and its
documentation. If the feature warrants a new page in the documentation, propose a
[Diátaxis](https://diataxis.fr) category for the page. A Starbase maintainer will
review your proposal and, if everything looks complete, assign the issue to you.

Certain high-value issues are allocated monetary bounties. If you're interested in
taking one on, we welcome you to apply.

### Create a development branch

Once you've settled on a topic to work on, it's time to set up a local branch.

Always start by syncing against the branch and the dependencies you're basing your
changes on.

If you're making a change to a current release, run:

```bash
git fetch upstream
git checkout -b <new-branch-name> upstream/hotfix/<current-release>
make setup
```

If you're contributing to multiple releases or the next major release, run:

```bash
git checkout main
git pull upstream main
git checkout -b <new-branch-name>
make setup
```

The new branch name should be brief, at no more than 80 characters. Format your branch
name as `<ticket-id>-<description>`. For example, if you're working on GitHub issue
\#235, and it's about adding a string sanitizer, you'd name your branch
`issue-235-add-string-sanitizer-method`.

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
> commit. This can happen with `test` and `docs`, which can be used as either types or
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

All nontrivial code changes should be accompanied by a reasonable set of tests.

<Update the next sentence to accurately describe your project's test suite.>

Starbase's test suite includes unit, integration, and
[Spread](https://github.com/canonical/spread/blob/master/README.md) tests. If you're not
sure which tests you should add, go with your best judgement – additional tests can be
added during the review process.

For low-complexity changes that require basic testing, run the fast tests:

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
rm -rf .venv
make clean
make setup
```

There are also special tests that are feature-specific. Run `make help` to view all of
them.

### Document the change

Before you start documenting your changes, take a moment to familiarize yourself with
the four categories of [Diátaxis](https://diataxis.fr), the framework that Starbase's
documentation is built around.

Most small changes call for updates to the existing pages that describe the current
behavior. Look for how-to guides and references that mention the affected feature, and
make any necessary changes.

Major changes require new documentation describing the feature's usage and
specifications. For example, if you implement a new CLI command, describe its usage in
one of the how-to guides and create a new reference for its options and flags.

<Ensure that the link points to your project's changelog or release notes.>

Ensure that feature changes and fixes are also documented in the relevant [release
notes](docs/release-notes/).

Once you've updated the documentation, build the site locally with:

```bash
make docs
```

Check for problems in the documentation with:

```bash
make lint-docs
```

### Push the branch and open a PR

Once your work is committed to your branch, push it to your fork:

```bash
git push -u origin <branch-name>
```

Finally, [open a PR](https://github.com/canonical/starbase/compare) for it on GitHub.
If your branch has one commit, GitHub will title the PR after it. If your branch has
more than one commit, name the PR after the most significant. Once open, reviewers are
assigned automatically to your work.

### Follow up for the review

The Starbase maintainers try to review every PR in a timely manner, typically within a
week for contributions that resolve an approved issue. While they can't guarantee
immediate feedback, they aim to ensure that all contributions are reviewed thoroughly
and thoughtfully.

When a maintainer reviews your PR, they typically leave inline comments and suggestions
on the code.

If the comment is a request, accommodate it by pushing one or more additional commits to
the branch. It's simplest to add the commits locally and push, rather than in the GitHub
interface, as it leads to fewer potential conflicts with syncs.

Don't force-push further changes to the branch after your PR is fully approved. It makes
it harder for reviewers to see whether any additional changes were made.
