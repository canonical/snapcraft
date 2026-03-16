.. meta::
    :description: How to make a documentation change in Snapcraft. Develop, test, and review documents in the standard workflow.

:relatedlinks: https://diataxis.fr, [Ubuntu&#32;Style&#32;Guide](https://docs.ubuntu.com/styleguide), [Starcraft&#32;style&#32;guide](https://canonical-starflow.readthedocs-hosted.com/how-to/starcraft-style-guide), [reStructuredText&#32;syntax&#32;reference](https://canonical-starter-pack.readthedocs-hosted.com/stable/reference/rst-syntax-reference)


.. _contribute-to-this-documentation:
.. _contribute-documentation:

Contribute to this documentation
================================

This guide describes how to make a change to the documentation in Snapcraft.

Documenting involves running commands in a command-line interface and syncing code with
Git and GitHub. If you're new to these tools, we recommend you make your first
contribution to Snapcraft with the help of :ref:`contribute-coda`.


Set up your work environment
----------------------------

Snapcraft documentation and development use the same build and test environment. If
you've already set up your local machine to develop Snapcraft, skip to the next section.

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

    make docs-setup
    make docs-lint

If these commands complete without error, your environment is ready.


Choose a task
-------------

Tasks come in different sizes and complexity. It's important to choose a task that you
have the capacity to finish, and plan accordingly.

Small changes and changes that need no explanation, like fixing spelling and grammar
mistakes or misplaced punctuation, are simple to incorporate and don't need any
planning.

If you intend to make a large or complex change, like a page rewrite or a bulk edit, the
task must be tracked as a `GitHub issue
<https://github.com/canonical/snapcraft/issues>`__. If no issue exists for your task,
`create one <https://github.com/canonical/snapcraft/issues/new/choose>`__ for it.

Once you choose an issue, volunteer for it in the issue's comments. A maintainer will
review your request and assign it to you.


Draft your work
---------------


Create a branch
~~~~~~~~~~~~~~~

Before you begin your work, sync your local copy of the Snapcraft code and create a new
branch.

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


.. _contribute-documentation-write:

Write
~~~~~

Snapcraft has conventions for documentation tone, style, accessibility, and formatting.
They are detailed in the related links on this page. If you're looking for a way to
express something that isn't covered by a convention, explore the existing documents.
They provide a broad base of effective technical writing.

All documents are in the ``docs`` directory. Pay attention to how the directory
structure, the document file names, and ``index.rst`` files combine to form the document
architecture.

If you're adding a page, rewriting a page, or making changes to multiple files, note
your task in the *Documentation* section of the version's release notes.


.. _contribute-documentation-test:

Test
~~~~

When you're ready to preview your draft, save all the files you worked on and build
the site locally:

.. code-block:: bash

    make docs

If successful, the terminal prints:

.. terminal::

    build succeeded.
    The HTML pages are in docs/_build.

Preview your changes in a web browser. Make sure the elements you've changed render as
expected. Double-check nested elements, such as content inside tabs and admonitions.

If everything looks good, check for problems in your document sources:

.. code-block:: bash

    make docs-lint


Commit
~~~~~~

Register the changes to your branch with a Git commit:

.. code-block:: bash

    git add -A
    git commit

Format the commit message like this:

    docs: <what you changed>

Keep the message short, at 80 characters or less, so other contributors and the project
maintainers can see the gist of what you did.

Commit early and often. It's normal to make multiple commits for a single piece of work,
especially when you come back to review it later. It's a good practice to get into to
keep your changes safe.

Committing triggers the pre-commit hook, which runs autoformatters. If any files were
autoformatted, re-add them and redo the commit.


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


Address quality concerns
~~~~~~~~~~~~~~~~~~~~~~~~

Before the PR is merged, it must pass all automatic checks, and it needs separate
approvals from two maintainers.

If there are any issues in your branch that your local testing didn't catch, then the
automatic checks will fail. To address these issues, review the logs in the failed
checks. The error messages in the logs will have remedies and hints for what needs
fixing.

When the maintainers review the PR, they may suggest improvements. Address them in
follow-up commits to your branch, the same way you committed and pushed changes while
drafting. If you feel a particular point should go in a different direction than what
they suggest, discuss it with the maintainer in the PR. They'll be happy to explore
alternatives.


Wrap up the review
~~~~~~~~~~~~~~~~~~

Once all suggestions are addressed, both maintainers will approve the PR. **After the PR
is approved, there may be a delay before merge.** The maintainers might need time to
coordinate the PR with other active developments.

After approval, **don't** force-push to your branch. It's difficult for the maintainers
to see whether any additional changes mixed into the push.

Once the PR is merged, your work is complete.


Get help and support
--------------------

Open source contribution can be difficult. Even the most experienced writers become
tangled or have moments of uncertainty.

If you're stuck, or need more information about a task, ask the issue creator or a
maintainer. If you need hands-on help, ask in the `Snapcraft Matrix channel
<https://matrix.to/#/#snapcraft:ubuntu.com>`__.
