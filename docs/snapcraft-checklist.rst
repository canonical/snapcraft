.. 10926.md

.. _snapcraft-checklist:

Snapcraft checklist
===================

Before you can build a snap, you need to know a few attributes about your application.

These attributes ensure a snap can be built and, via a 3-point checklist outlined below, help with the construction of the application’s :file:`snapcraft.yaml`.

Prerequisites
-------------

There are currently a few types of project that are unsuitable for snaps:

- libraries
- media content providers (except themes)
- applications built specifically for ARMv6
- applications that need a binary in ``/snap/core18/current/{bin,usr/bin}/*``

   Due to Snapcraft’s flexibility, there are exceptions to the above. These include :ref:`FFmpeg <the-ffmpeg-sdk-stage-snaps>` and :ref:`wxWidget <the-wxwidgets-sdk-stage-snaps>` stage snaps, and the :ref:`Hunspell dictionaries <the-hunspell-dictionaries-content-snaps>` snap.

Before going any further, make sure your project builds and runs from a clean environment. This will help clarify any wayward dependencies or specific installation requirement that may have been forgotten in an old build tree.

Creating a checklist
--------------------

A snap’s requirements reflect those of the application itself.

If you’re a developer working on the application, or a technical user familiar with a project, a snap’s requirements won’t contain any surprises; they’re what you need to build your application.

Split roughly into their relevance, and the order you should tackle each requirement, here’s what you need to know:

1. **Language/Framework/Build system** (*mandatory*)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is the foundation of your snap. It defines how your snap is built and is often an extension of the programming language you’re using. Examples include Python applications using `PyPI <https://pypi.org/>`__ and Go projects using `go get <https://golang.org/pkg/cmd/go/internal/get/>`__, but also build systems like :ref:`cmake <the-cmake-plugin>` and platforms like :ref:`Electron <electron-apps>`.

Snapcraft uses :ref:`plugins <snapcraft-plugins>` to create the build environment, and there are plugins for many of the most popular build systems. See :ref:`Supported plugins <supported-plugins>` for more details.

   Applications can be built using a single part with a single plugin, or from multiple parts and multiple plugins, depending on their complexity.

2. **Toolkits and desktop support** (*optional*)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Many applications use a toolkit, such as `Qt <https://www.qt.io/>`__ or `GTK <https://www.gtk.org/>`__, to provide both functionality and better system integration.

There are recipes for incorporating many popular toolkits into your snap, either by pasting pre-configured snippets into your snap’s :file:`snapcraft.yaml` file, or by using a new Snapcraft feature called :ref:`Extensions <snapcraft-extensions>`.

See :ref:`Desktop app support <desktop-applications>` for examples with toolkit and desktop integration.

3. **System integration** (*optional*)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Your application may have requirements of the system it’s running on, and these requirements are typically satisfied by configuring one or more :ref:`interfaces <interface-management>`.

These requirements may be as simple as access to a user’s :ref:`home directory <the-home-interface>`, sound :ref:`playback <the-audio-playback-interface>` and :ref:`recording <the-audio-record-interface>` via PulseAudio and :ref:`desktop interfaces <the-desktop-interfaces>`. But they can equally include :ref:`process management <the-process-control-interface>` or :ref:`memory access <the-physical-memory-observe-interface>`.
