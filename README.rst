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
TODO

Create a new project
---------------------------
[TODO: Make this a template repository.]

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
