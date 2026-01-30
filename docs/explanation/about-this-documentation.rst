.. _explanation-about-this-documentation:

About this documentation
========================

The documentation is an essential part of Snapcraft. We make documentation a disciplined
and principled part of engineering. It's as much of a priority for the project as
feature development.


Open source contributions
-------------------------

Snapcraft is open source and made possible by the community. We accommodate changes,
suggestions, and questions from everyone. That includes you!

Getting involved with the documentation is an opportunity to grow and learn. You'll
practice technical writing and product testing, which are valuable personal and
professional skills. You'll have a chance to work very visibly with industry
professionals and technical authors on interesting problems, and add to your
professional portfolio.

Every contributor is recognized in the Snapcraft releases that they took part in.


Types of contributions
----------------------

There are multiple ways you can make a valuable contribution to the documentation:

- :ref:`Add to and improve <how-to-contribute-to-this-documentation>` the documentation
- `Share feedback or report a problem
  <https://github.com/canonical/snapcraft/issues/new/choose>`_ with a page
- Answer documentation questions on the `Snapcraft forum <https://forum.snapcraft.io>`_
- Help community members in the `Snapcraft Matrix channel
  <https://matrix.to/#/#snapcraft:ubuntu.com>`_
- Work on tasks for Snapcraft in :ref:`explanation-about-this-documentation-coda`


Contribution excellence
-----------------------

We strive to make our documentation excellent. The expectations for all contributors are outlined in these documents:

- `Ubuntu Code of Conduct <https://ubuntu.com/community/ethos/code-of-conduct>`_.
  Projects governed by Canonical expect good conduct and excellence from every
  contributor. Our principles are laid out in this document.
- `Canonical Contributor License Agreement <http://www.ubuntu.com/legal/contributors>`_.
  As a contributor, you retain your copyright and attribution rights, provided you sign
  the agreement. Before committing any code, review its terms. If you agree and sign it,
  your code can be incorporated into the repository.


.. _explanation-about-this-documentation-coda:

Canonical's Open Documentation Academy
--------------------------------------

Canonical's Open Documentation Academy (CODA) is a space where writers and developers
give hands-on help and mentoring to anyone wanting to participate in open source. CODA
coordinates documentation work for open-source projects from all over, including
Snapcraft. It's an initiative led by the documentation team at Canonical.

If you've never contributed to open source, CODA is the best way to start. It provides:

- A space to debut in the open source community
- Guidance in choosing a first task in Snapcraft
- Direct support for contributions in Snapcraft and other projects
- Skill development in documentation and writing

The best way to get started is to introduce yourself in the `CODA Matrix channel
<https://matrix.to/#/#documentation:ubuntu.com>`_ and volunteer for a topic in the `CODA
task aggregator
<https://github.com/canonical/open-documentation-academy/issues?q=is%3Aissue%20state%3Aopen%20snapcraft>`_.


Documentation system and process
--------------------------------

Snapcraft practices docs-as-code. The document source files are written in
human-friendly reStructuredText markup, and kept in the ``docs`` directory inside the
Snapcraft source code. Like the rest of the code, the documents are version-controlled
in a Git repository and hosted on GitHub.

The project uses Sphinx to compile the document sources into a static website of HTML
web pages. Most of the work of documenting takes place on your local machine. When
developing, the documents you render locally are the same as they would be on the
website.

The published documentation is hosted on the Read the Docs platform.

Every time the source code is changed on GitHub, the documentation for that state of the
software is built and published. This is how a new copy of the documentation is provided
for each release.

Snapcraft uses a forking, feature-branch workflow. Documentation changes are made in
branches on the author's fork, and then proposed for merge in a pull request on the main
repository.

Writing and editing in the docs-as-code style follows a write-build-preview loop.

The Snapcraft maintainers try and review every PR in a timely manner, typically within a
week for PRs that complete an assigned issue. They aim to ensure that all contributions
are reviewed thoroughly and thoughtfully.

During review, we recommend not using the GitHub interface to address code suggestions.
It's simplest to add the commits locally and push, as it avoids conflicts with
authorship and syncs.

While the maintainers appreciate a clean history, it's not mandatory, since the PR's
commits are squashed before merge.


Writing styles and conventions
------------------------------

There is no single way to write, but there are guidelines and patterns that Snapcraft
documents follow:

- `Diátaxis <https://diataxis.fr>`_
- `Canonical Style Guide <https://docs.ubuntu.com/styleguide>`_
- `Inclusive terms
  <https://github.com/canonical/Inclusive-naming/blob/main/config.yml>`_
- `reStructuredText syntax reference
  <https://canonical-starter-pack.readthedocs-hosted.com/stable/reference/rst-syntax-reference>`_
