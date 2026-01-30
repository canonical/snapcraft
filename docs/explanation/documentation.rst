.. meta::
  :description: An explanation of the documentation system, process, and writing style and conventions in Snapcraft.


.. _explanation-documentation:

About this documentation
========================

The documentation is an essential part of Snapcraft. We make documentation a disciplined
and principled part of engineering with its own architecture and quality standards.


Documentation system and process
--------------------------------

Snapcraft practices docs-as-code. The document source files are written in
reStructuredText markup and kept inside the Snapcraft source code. Like the rest of the
code, the documents are version-controlled in a Git repository and hosted on GitHub.

The project uses Sphinx to compile the document sources into a static website of HTML
web pages. The published documentation is hosted on the Read the Docs platform.

Every time the source code is changed on GitHub, the documentation for that state of the
software is built and published. This is how a new copy of the documentation is provided
for each release.

Writing and editing in the docs-as-code style follows a write-build-preview loop.

The Snapcraft maintainers try and review every PR in a timely manner, typically within a
week for PRs that complete an assigned issue. They aim to ensure that all contributions
are reviewed thoroughly and thoughtfully.


Writing styles and conventions
------------------------------

There is no single way to write, but there are guidelines and patterns that Snapcraft
documents follow:

- `Diátaxis <https://diataxis.fr>`__
- :external+starflow:ref:`how-to-starcraft-style-guide`
- `Canonical Style Guide <https://docs.ubuntu.com/styleguide>`__
- `Inclusive terms
  <https://github.com/canonical/Inclusive-naming/blob/main/config.yml>`__
- `reStructuredText syntax reference
  <https://canonical-starter-pack.readthedocs-hosted.com/stable/reference/rst-syntax-reference>`__
