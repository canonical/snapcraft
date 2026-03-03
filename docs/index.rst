.. meta::

    :description: Snapcraft is the command-line build tool for packaging and distributing software and apps in the snap container format.


Snapcraft
=========

**Snapcraft** is the command-line build tool for packaging and distributing software and
apps in the snap container format.

The tool packages apps across many supported languages, build tools, and frameworks,
such as Python, C and C++, Rust, Node, and GNOME. Snaps can be tested, debugged, and
locally shared before being published to the global Snap Store and private stores. It
uses simple commands to manage and monitor releases at a granular level.

It solves the problems of dependency management and architecture support by bundling all
of a software's libraries into the container itself, and provides a way to package any
app, program, toolkit, or library for all major Linux distributions and IoT devices.

Snapcraft is for developers, package maintainers, fleet administrators, and hobbyists
who are interested in publishing snaps for Linux and IoT devices.


In this documentation
---------------------

.. list-table::
    :widths: 35 65
    :header-rows: 0

    * - **Tutorial**
      - :ref:`tutorial-craft-a-snap-define-package-information` •
        :ref:`tutorial-craft-a-snap-define-the-target-platforms` •
        :ref:`tutorial-craft-a-snap-define-the-main-part` •
        :ref:`tutorial-craft-a-snap-pack-the-snap` •
        :ref:`tutorial-craft-a-snap-test-the-snap` •
        :ref:`tutorial-craft-a-snap-connect-the-interfaces` •
        :ref:`tutorial-craft-a-snap-secure-the-snap`
    * - **Installation and setup**
      - :ref:`reference-system-requirements` • :ref:`how-to-set-up-snapcraft` •
        :ref:`how-to-select-a-build-provider`
    * - **Vocabulary and syntax**
      - :ref:`reference-commands` • :ref:`reference-snapcraft-yaml` •
        :ref:`how-to-configure-package-information`
    * - **Platform compatibility**
      - :ref:`explanation-bases` • :ref:`how-to-specify-a-base` •
        :ref:`explanation-platforms` • :ref:`how-to-select-platforms` •
        :ref:`reference-advanced-grammar`
    * - **Software integration**
      - :ref:`how-to-integrations` • :ref:`explanation-parts` • :ref:`reference-plugins`
        • :ref:`reference-package-repositories` • :ref:`how-to-manage-dependencies` •
        :ref:`explanation-extensions`
    * - **Sandboxing and access control**
      - :ref:`explanation-interfaces` • :ref:`reference-layouts` •
        :ref:`reference-build-environment-options` • :ref:`reference-hooks`
    * - **Snap migration**
      - :ref:`how-to-change-from-core18-to-core20` •
        :ref:`how-to-change-from-core20-to-core22` •
        :ref:`how-to-change-from-core22-to-core24`
    * - **Debugging**
      - :ref:`reference-linters` • :ref:`how-to-debug-a-snap`
    * - **Distribution**
      - :ref:`reference-channels` • :ref:`how-to-register-a-snap` •
        :ref:`how-to-publish-a-snap` • :ref:`how-to-manage-revisions-and-releases`


How this documentation is organized
-----------------------------------

The Snapcraft documentation embodies the `Diátaxis framework <https://diataxis.fr/>`__.

* The :ref:`tutorial <tutorials>` is a lesson that steps through the main process of
  packaging a snap.
* :ref:`how-to-guides` contain directions for crafting and debugging snaps.
* :ref:`References <reference>` describe the structure and function of the individual components in
  Snapcraft.
* :ref:`Explanations <explanation>` aid in understanding the concepts and relationships
  of Snapcraft as a system.


Project and community
---------------------

Snapcraft is a member of the Canonical family. It's an open source project that warmly
welcomes community projects, contributions, suggestions, fixes and constructive
feedback.


Get involved
~~~~~~~~~~~~

* `Snapcraft Matrix channel <https://matrix.to/#/#snapcraft:ubuntu.com>`__
* `Snapcraft forum <https://forum.snapcraft.io/>`__
* `Contribute to Snapcraft development <https://github.com/canonical/snapcraft/blob/main/CONTRIBUTING.md>`__
* :ref:`contribute-to-this-documentation`


Releases and support
~~~~~~~~~~~~~~~~~~~~

* :ref:`release-notes`
* :ref:`reference-support-schedule`


Governance and policies
~~~~~~~~~~~~~~~~~~~~~~~

* `Ubuntu Code of Conduct <https://ubuntu.com/community/docs/ethos/code-of-conduct>`__
* `Canonical Contributor License Agreement
  <https://ubuntu.com/legal/contributors>`__


.. toctree::
    :hidden:

    tutorials/index
    how-to/index
    reference/index
    explanation/index
    contribute-to-this-documentation
    release-notes/index
