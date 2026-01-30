:relatedlinks: https://diataxis.fr, [Ubuntu&#32;Style&#32;Guide](https://docs.ubuntu.com/styleguide), [Inclusive&#32;terms](https://github.com/canonical/Inclusive-naming/blob/main/config.yml), [reStructuredText&#32;syntax&#32;reference](https://canonical-starter-pack.readthedocs-hosted.com/stable/reference/rst-syntax-reference)

.. _contribute-to-this-documentation:
.. _how-to-contribute-to-this-documentation:

Contribute to this documentation
================================

This guide describes how to make a change to the documentation in Snapcraft.

Changing the documentation involves:

- Running commands in a command-line interface or terminal
- Writing in reStructuredText
- Saving changes in Git
- Syncing changes with a personal GitHub account

If you're unfamiliar with any of these activities, we recommend you first build some
experience in :ref:`explanation-about-this-documentation-coda`.


Documentation checklist
-----------------------

.. |checkbox| raw:: html

    <input type="checkbox">

**Start**

|checkbox| Created or volunteered for issue on GitHub

|checkbox| Local code in sync

|checkbox| New branch created

**Write**

|checkbox| Changes made to pages and indexes

|checkbox| New pages, rewritten pages, and broad edits documented in release notes

|checkbox| Docs pass all local checks

|checkbox| Changes committed

|checkbox| Branch pushed to fork

**Review**

|checkbox| Pull request opened

|checkbox| Docs pass all automatic checks

|checkbox| All review comments addressed

|checkbox| Pull request approved by two maintainers

|checkbox| Pull request merged


Set up
------

Documentation and Snapcraft features are developed in the same environment. If you've
previously set up your local machine to develop Snapcraft features, skip to the next
section.

Start by `creating a personal fork <https://github.com/canonical/snapcraft/fork>`_ of
the repository on GitHub.

Next, clone your fork onto your local machine.

.. tab-set::

    .. tab-item:: With SSH

        If you authenticate your GitHub account with `SSH
        <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>`_,
        run:

        .. code-block:: bash

            git clone git@github.com:<username>/snapcraft --recurse-submodules
            cd snapcraft
            git remote add upstream git@github.com:canonical/snapcraft
            git fetch upstream

    .. tab-item:: With HTTPS

        If you don't authenticate with SSH, clone with `HTTPS
        <https://docs.github.com/en/get-started/git-basics/about-remote-repositories#cloning-with-https-urls>`_
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

If these commands complete without error, your local environment is ready.


Choose a task
-------------

Tasks come in different sizes and complexity. It's important to choose a task that you
have the capacity to finish, and plan accordingly.

Small changes and changes that need no explanation, like fixing spelling and grammar
mistakes or misplaced punctuation, are simple to incorporate and don't need any
planning.

If you intend to make a large or complex change, like a page rewrite or a bulk edit, the
task must be represented in an open GitHub issue. Check the `Snapcraft issues
<https://github.com/canonical/snapcraft/issues>`_ to see whether someone has planned
work along the lines of your change. If not, `create a ticket
<https://github.com/canonical/snapcraft/issues/new/choose>`_ for it. Once you have the
issue for your work, indicate your interest in the issue's comments. A maintainer will
review the issue and assign it to you.


Create a branch
---------------

Before you can begin writing, sync your local copy of the Snapcraft code and create a
new branch.

.. tab-set::

    .. tab-item:: Current releases

        If you're contributing to a current release, run:

        .. code-block:: bash

            git fetch upstream
            git switch -c <new-branch-name> upstream/hotfix/<current-release>
            make docs-setup

    .. tab-item:: Future releases

        If you're contributing to the next major release, run:

        .. code-block:: bash

            git switch main
            git pull upstream main
            git switch -c <new-branch-name>
            make docs-setup

If your task has no GitHub issue, make the branch name succinct and memorable, so that
you can keep track of your different branches. A useful format is to start the name with
``docs-`` followed by a few keywords to describe the work. For example, if the task is
to add a missing comma in the tutorial, you could name the branch
``docs-tutorial-add-comma``.

If the task has a GitHub issue, start with the ticket ID instead. Starting with the ID
is especially helpful when you're addressing the issue across multiple branches. For
example, if you're working on GitHub issue #2135, and updating the command reference,
you could name the branch ``issue-2135-command-reference``.


Write and test
--------------

Next, make your changes to the docs in the ``docs`` directory. There are writing styles
and patterns that Snapcraft follows. If you need guidance on the writing conventions,
explore the related links on this page.

If you're adding a page, rewriting a page, or making changes to multiple files, document
your work as an entry in the *Documentation* section of the version's release notes.

When you're ready to preview your changes, save the files you worked on, then build the
site locally:

.. code-block:: bash

    make docs

If successful, the terminal will read:

.. code-block:: bash

    The HTML pages are in _build.
    [sphinx-autobuild] Serving on http://127.0.0.1:8000
    [sphinx-autobuild] Waiting to detect changes...

Preview your changes in a web browser. Make sure the elements you've changed render as
expected. Double-check nested elements, such as content inside tabs and admonitions.

If everything looks good, check for problems in your code:

.. code-block:: bash

    make docs-lint


Commit
------

Documentation changes are only made permanent through a Git commit:

.. code-block:: bash

    git add -A
    git commit

It's normal to make multiple commits for a single piece of work, especially when you
come back to review it later. Commit early and often. It's a good practice to get into
to keep your changes safe.

Format the commit message with the ``docs`` conventional commit type:

    docs: <what you changed>

Keep the message short, at 80 characters or less, so other contributors and the project
maintainers can see the gist of what you did.

Committing triggers the pre-commit hook, which runs autoformatters. If any files were
autoformatted, re-add them and redo the commit.


Send for review
---------------

When you've committed all your work and you're ready to have it reviewed, push it to
your fork:

.. code-block:: bash

    git push -u origin <branch-name>

Then, `open a pull request (PR) <https://github.com/canonical/snapcraft/compare>`_ for
it on GitHub. Give the PR a title using the same message format as the commit. If your
branch has more than one commit, reuse the message from the first.


Complete the review
-------------------

Before your PR is merged, it needs to pass all automatic checks, and two separate
approvals from two maintainers.

If there are any issues in your branch that your local testing didn't catch, then the
automatic checks will fail. To address these issues, review the logs in the failed
checks. The error messages in the logs will have remedies and hints for what needs
fixing.

When the maintainers review your PR, they may suggest improvements to your code. Address
them in follow-up commits to your branch, the same way you committed and pushed changes
while writing. If you feel a particular point should go in a different direction than
what they suggest, discuss it with the maintainer in the suggestion's thread. They're
happy to explore alternatives.

Once all suggestions are addressed, both maintainers approve your PR, and soon after
it's merged. **After your PR is approved,** **there may be a delay before merge**. The
maintainers may need time to coordinate your PR with other development on the project.
While the PR is active, **don't** force-push to your branch. It makes it harder for the
maintainers to see whether any additional changes were made.

Once your PR is approved and merged, your contribution is complete.


Help and support
----------------

Open source contribution can be difficult. Even the most experienced writers become
tangled or have moments of uncertainty.

If you're stuck, or need more information about a task, ask the issue creator, the
project's technical author, or a maintainer. If you need hands-on help, ask in the
Snapcraft Matrix channel.
