.. meta::
    :description: How to make a code change in Snapcraft. Code, test, and review changes in the project within the standard workflow.

:relatedlinks: [Pytest&#32;test&#32;pattern](https://docs.pytest.org/en/stable/explanation/anatomy.html), [Conventional&#32;Commits](https://www.conventionalcommits.org/en/v1.0.0/), [Spread](https://github.com/canonical/spread/blob/master/README.md)


.. _contribute-development:

Contribute to development
=========================

This guide describes how to contribute a code change in Snapcraft.

It requires familiarity with Python, Git, and GitHub.


Set up your work environment
----------------------------

Snapcraft development and documentation use the same build and test environment. If you
previously set up your local machine to document Snapcraft, skip to the next section.

For security, you must sign your commits. If you haven't already, `configure Git to sign
with your GPG or SSH key
<https://docs.github.com/en/authentication/managing-commit-signature-verification/telling-git-about-your-signing-key>`__.

`Create a personal fork <https://github.com/canonical/snapcraft/fork>`__ of the
repository on GitHub.

Next, clone your fork onto your local machine:

.. tab-set::

    .. tab-item:: With SSH

        If you authenticate your GitHub account with `SSH
        <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>`__,
        run:

        .. code-block:: bash

            git clone git@github.com:<username>/snapcraft --recurse-submodules
            cd snapcraft
            git remote add upstream git@github.com:canonical/snapcraft
            git fetch upstream

    .. tab-item:: With HTTPS

        If you don't authenticate with SSH, clone with `HTTPS
        <https://docs.github.com/en/get-started/git-basics/about-remote-repositories#cloning-with-https-urls>`__
        instead:

        .. code-block:: bash

            git clone https://github.com/<username>/snapcraft --recurse-submodules
            cd snapcraft
            git remote add upstream https://github.com/canonical/snapcraft
            git fetch upstream

Inside the project directory, set up the development environment:

.. code-block:: bash

    make setup
    make lint
    make test-fast

If these commands complete without error, your environment is ready.


Choose a task
-------------

Every task must be tracked as a `GitHub issue
<https://github.com/canonical/snapcraft/issues>`__. If no issue exists for your task,
`create one <https://github.com/canonical/snapcraft/issues/new/choose>`__ for it.

Tasks come in different sizes and complexity. It's important to choose a task that you
have the capacity to finish. It's also important to check to see if the task is already
planned. You might save time and effort by checking for prior work.


Simple tasks
~~~~~~~~~~~~

For obvious tasks that need no explanation, like fixing a lint failure, volunteer for
its GitHub issue. A maintainer will review your request and assign it to you.


Complex tasks
~~~~~~~~~~~~~

For complex tasks, like a new module or class, propose your solution in its GitHub
issue. Describe an implementation, tests, and documentation. If your plan is feasible, a
Snapcraft maintainer will assign the issue to you.

For very complex tasks, like refactors, contact the team in the `Snapcraft Matrix
channel <https://matrix.to/#/#snapcraft:ubuntu.com>`__.

For tasks that require coordination with the Snap Store or snapd, create a post on the
`forum <https://forum.snapcraft.io>`__, which serves as a central point for all teams
working on snaps.


Draft your work
---------------


Create a branch
~~~~~~~~~~~~~~~

Before you begin your work, sync your local copy of the Snapcraft code and create a new
branch.

Always start by syncing against the branch and the dependencies you're basing your
changes on:

.. tab-set::

    .. tab-item:: Current releases

        If you're contributing to a current release, run:

        .. code-block:: bash

            git fetch upstream
            git switch -c <new-branch-name> upstream/hotfix/<current-release>
            make setup

    .. tab-item:: Future releases

        If you're contributing to multiple releases or the next major release, run:

        .. code-block:: bash

            git switch main
            git pull upstream main
            git switch -c <new-branch-name>
            make setup

Format your branch name as ``<ticket-id>-<description>``. For example, if you're working
on GitHub issue #235, and it's about adding a string sanitizer, you'd name your branch
``issue-235-add-string-sanitizer``.


Develop
~~~~~~~

Snapcraft has conventions for its code style and testing. They are detailed in the
related links on this page. If you're looking for a way to develop something that isn't
covered by a convention, explore the existing codebase.

The Snapcraft codebase is large. You might be tempted to tweak related behavior or fix
tangential inconsistencies. Don't. Stay focused on your task and keep its scope narrow.

Most development tasks involve adding or changing algorithms. The architecture, meaning
the organization of the code and the design of its structures, changes much less often.
If in your work you need to change structures or reorganize the code, consult the team
first.

Snapcraft has 10-year long-term support (LTS) commitments. One of our most important
reliability principles is that a snap on an LTS core, if unaltered, must rebuild without
intervention for the duration of the LTS period. All changes you make to Snapcraft must
be backward-compatible unless stated otherwise in the issue.

All nontrivial changes must be accompanied by new or updated tests. The Snapcraft test
suite includes both pytest unit tests and Spread integration tests. When adding unit
tests, follow the arrange-act-assert-cleanup pattern from Pytest. Tests are rarely
perfect on the first try. Additional tests can always be added during the review
process.

Snapcraft is amenable to code generated with LLM assistance. Generating code is a time
saver, but like all code it needs testing and careful review. An LLM doesn't absolve you
of responsibility -- you are ultimately responsible for the code in your work.


Commit
~~~~~~

Register the changes to your branch with a Git commit:

.. code-block:: bash

    git add -A
    git commit

Format the commit message as a `conventional commit
<https://www.conventionalcommits.org/en/v1.0.0/>`__. For example, a new feature would be
written as:

    feat: <description of new feature>

Keep the message short, at 80 characters or less, so other contributors and the project
maintainers can see the gist of what you did.

Commit early and often. It's normal to make multiple commits for a single piece of work,
especially when you come back to review it later. It's a good practice to get into to
keep your changes safe.

Committing triggers the pre-commit hook, which runs autoformatters. If any files were
autoformatted, re-add them and redo the commit.

.. admonition:: Committing complex changes
    :class: tip

    With complex changes, you might get stuck choosing a conventional commit type.

    This may signal that a commit is doing more than one thing and should be broken into
    multiple smaller commits. A commit should not, for example, refactor code and fix a
    bug. That should be two separate commits.

    In other scenarios, multiple types could be appropriate because of the nature of the
    commit. This can happen with ``test`` and ``docs``, which can be used as either
    types or scopes.

    When combining commit types, select the highest-ranked type that fits:

    - :vale-ignore:`ci`
    - build
    - feat
    - fix
    - :vale-ignore:`perf`
    - refactor
    - style
    - test
    - docs
    - chore


Test
~~~~

All code changes are subjected to local and remote testing.

**If a test breaks, it usually means your code made a breaking change**. Diagnose what
went wrong and why, and iterate until it passes.

For low-complexity changes that require basic testing, run the fast suite, which takes a
few seconds:

.. code-block:: bash

    make test-fast

For complex work, run the full test suite, which takes several minutes:

.. code-block:: bash

    make test

When iterating and testing, it's a good practice to clean the local temporary files that
the tests generate:

.. code-block:: bash

    make clean

In rare instances, tests can fail in unpredictable ways, regardless of the state of your
code. If that happens, delete your virtual environment and start over:

.. code-block:: bash

    rm -rf .venv
    make clean
    make setup


There are also special tests that are feature-specific. Run ``make help`` to view and
select them.


Document your work
~~~~~~~~~~~~~~~~~~

Add all changes and fixes to the upcoming release notes.

Small changes usually require updates to the pages that describe the feature's behavior.
Make any necessary changes to how-to guides and references that cover the feature.

Major changes require new documentation describing the feature's usage and
specifications. For example, if you implement a new CLI command, describe its usage in
one of the how-to guides and add reference information for its options and flags.

The documentation contribution guide shows how to:

- :ref:`Write the docs <contribute-documentation-write>`
- :ref:`Test the docs <contribute-documentation-test>`


Review with the team
--------------------


Send for review
~~~~~~~~~~~~~~~

When you've committed your draft and you're ready to have it reviewed, push it to your
fork:

.. code-block:: bash

    git push -u origin <branch-name>

Then, `open a pull request (PR) <https://github.com/canonical/snapcraft/compare>`__ for
it on GitHub. Give the PR a title using the same message format as the commit. If your
branch has more than one commit, reuse the message from the first.

If you used an LLM to generate code, explain which parts are synthetic in the PR
description.


Address quality concerns
~~~~~~~~~~~~~~~~~~~~~~~~

Before the PR is merged, it must pass all automatic checks, and it needs separate
approvals from two maintainers.

If there are any issues in your branch that your local testing didn't catch, then the
automatic checks will fail. To address these issues, review the logs in the failed
checks. The error messages in the logs will have remedies and hints for what needs
fixing.

When the maintainers review the PR, they may suggest improvements to your code. Address
them in follow-up commits to your branch, the same way you committed and pushed changes
while drafting. If you feel a particular point should go in a different direction than
what they suggest, discuss it with the maintainer in the PR. They'll be happy to explore
alternatives.


Wrap up the review
~~~~~~~~~~~~~~~~~~

Once all suggestions are addressed, both maintainers approve the PR, and merge it soon
after. **After the PR is approved, there may be a delay before merge.** The maintainers
might need time to coordinate the PR with other development on the project.

After approval, **don't** force-push to your branch. It's difficult for the maintainers
to see whether any additional changes mixed into the push.

Once the PR is merged, your work is complete.


Get help and support
--------------------

Open source contribution can be difficult. Even the most experienced writers become
tangled or have moments of uncertainty.

If you're stuck, or need more information about a task, ask the issue creator, the
project's technical author, or a maintainer. If you need hands-on help, ask in the
Snapcraft Matrix channel.
