.. meta::
    :description: View the contribution process and ways to contribute to Snapcraft. Discover the expectations and benefits of contributing.


.. _contribute:

Contribute
==========

Snapcraft has a community from all over the world, and the Snapcraft team welcomes all
contributions.

Contributing offers an opportunity to polish your technical skills, develop as a
professional, and get involved in the growing open source community. Snapcraft
contributors are also recognized in any releases that they work on.

All contributors should become familiar with these guides. They outline the expectations
and practices for participating in the project.


Standards and expectations
--------------------------

The Starcraft team at Canonical sets the direction and priorities of Snapcraft. They
take responsibility for its stewardship and health.

Before you contribute to Snapcraft, there are three documents for you to digest.


Ubuntu Code of Conduct
~~~~~~~~~~~~~~~~~~~~~~

Projects governed by Canonical expect good conduct and excellence from every member. The
specific principles are laid out in the `Ubuntu Code of Conduct
<https://ubuntu.com/community/ethos/code-of-conduct>`__.


Canonical Contributor License Agreement
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You retain your copyright and attribution rights of the code you contribute, provided
you sign the `Canonical Contributor License Agreement
<https://www.ubuntu.com/legal/contributors>`__. Before committing any code, review its
terms. If you agree and sign it, your code can be incorporated into the repository.


Open source license
~~~~~~~~~~~~~~~~~~~

Snapcraft is licensed under `GPL-3.0
<https://github.com/canonical/snapcraft/blob/main/LICENSE>`__.


Ways to contribute
------------------

There are multiple ways you can make a valuable contribution:

- :ref:`Develop <contribute-development>` features and fix problems
- :ref:`Document <contribute-documentation>` Snapcraft
- :ref:`Share feedback or report a problem <contribute-feedback>`
- :ref:`Test upcoming features and fixes <contribute-test-builds>`
- Work on Snapcraft tasks in :ref:`contribute-coda`
- Answer questions on the `Snapcraft forum <https://forum.snapcraft.io>`__
- Help community members in the `Snapcraft Matrix channel
  <https://matrix.to/#/#snapcraft:ubuntu.com>`__


.. _contribute-feedback:

Feedback and issues
~~~~~~~~~~~~~~~~~~~

If you find a bug or feature gap in Snapcraft, look for it in the `project's GitHub
issues <https://github.com/canonical/snapcraft/issues>`__ first. If you have fresh
input, add your voice to the issue.

If the bug or feature doesn't have an issue, we invite you to `open one
<https://github.com/canonical/snapcraft/issues/new/choose>`__.


.. _contribute-test-builds:

Test builds
~~~~~~~~~~~

Major features and bug fixes can take time to develop. We gladly invite you to test
these builds.

Long-lived feature branches and PRs automatically create test builds of Snapcraft for
the AMD64 and ARM64 platforms. The test builds are available as snaps in special test
channels:

- ``latest/edge/pr-<PR-number>`` for PR snaps
- ``latest/edge/<branch-name>`` for feature branch snaps

To start, enable parallel snap instances on your host:

.. code-block:: bash

    sudo snap set system experimental.parallel-instances=true

Once you have the PR or branch you'd like to test, download and run its build as an
instance of Snapcraft:

.. code-block:: bash

    snap install snapcraft_<instance> --channel <channel-name>
    snapcraft_<instance>

Give the Snapcraft instance a unique name so that it doesn't collide with any other
instances on your host.


.. _contribute-coda:

Canonical's Open Documentation Academy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Canonical's Open Documentation Academy (CODA) is a space where writers and developers
give hands-on help and mentoring to anyone wanting to participate in open source. CODA
coordinates documentation work for open-source projects from all over, including
Snapcraft. It's an initiative led by the documentation team at Canonical.

Getting involved with documentation is an opportunity to grow and learn. You'll practice
technical writing and product testing, which are valuable personal and professional
skills. You'll have a chance to work very visibly with industry professionals and
technical authors on interesting problems, and add to your professional portfolio.

If you've never contributed to open source, CODA is the best way to start. It provides:

- A space to debut in the open source community
- Guidance in choosing a first task in Snapcraft
- Direct support for contributions in Snapcraft and other projects
- Skill development in documentation and writing

To get started, introduce yourself in the `CODA Matrix channel
<https://matrix.to/#/#documentation:ubuntu.com>`__ and volunteer for a topic in the
`CODA task aggregator
<https://github.com/canonical/open-documentation-academy/issues?q=is%3Aissue%20state%3Aopen%20snapcraft>`__.


.. toctree::
    :hidden:

    Development <development>
    Documentation <documentation>
