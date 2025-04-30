.. snapcraft documentation root file

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

.. list-table::

    * - | :ref:`Tutorial <tutorials>`
        | Get started with a hands-on introduction to building snaps.
      - | :ref:`How-to guides <how-to-guides>`
        | Step-by-step guidance for tasks in :ref:`crafting
          <how-to-configure-package-information>`,
          :ref:`debugging <how-to-debug-a-snap>`, and :ref:`publishing
          <how-to-authenticate>` snaps.
    * - | :ref:`Reference <reference>`
        | Technical information about Snapcraft, from :ref:`commands
          <reference-commands>` to :ref:`plugins <reference-plugins>`.
      - | :ref:`Explanation <explanation>`
        | Discussion and clarification of key concepts, such as :ref:`architectures
          <explanation-architectures>`, :ref:`bases <explanation-bases>`, and
          the :ref:`parts lifecycle <explanation-parts-lifecycle>`.


Project and community
---------------------

Snapcraft is a member of the Canonical family. It's an open source project that warmly
welcomes community projects, contributions, suggestions, fixes and constructive
feedback.

- `Ubuntu Code of Conduct <https://ubuntu.com/community/code-of-conduct>`_
- `Canonical contributor license agreement <https://ubuntu.com/legal/contributors>`_

.. toctree::
    :hidden:

    tutorials/index
    how-to/index
    reference/index
    explanation/index
    release-notes/index
