.. _reference-external-package-information:

External package information
============================

To help avoid duplication, Snapcraft can process and incorporate external package
information from within a project file.

This reference describes the available external information sources. For a guide on how
to use them, see  :ref:`how-to-configure-package-information`.


Source Types
------------

An external metadata source can be one of the following:

- :ref:`AppStream <reference-external-package-appstream>`: a standard for software
  components
- :ref:`Scriptlets <reference-external-package-scriptlets>`: a craftctl-driven
  command to generate ``version`` and ``grade``.


.. _reference-external-package-appstream:

AppStream
~~~~~~~~~

`AppStream`_ is a metadata standard used to describe a common set of software
components. It can be parsed by Snapcraft to provide the ``title``, ``version``,
``summary``, ``description``, and ``icon`` keys for a snap, along with the location of
an app's :ref:`desktop <how-to-configure-package-information-desktop-entry-copy-file>`
file.

The following is a typical example from an upstream project.

.. code-block:: xml
    :caption: sampleapp.metainfo.xml

    <?xml version="1.0" encoding="UTF-8"?>
    <component type="desktop-application">
        <id>com.example.sampleapp</id>
        <name>Sample App</name>
        <project_license>GPL-3.0+</project_license>
        <name>Sample App</name>
        <summary>Single-line elevator pitch for your amazing application</summary>
        <description>
            This is applications's description. A paragraph or two to tell the
            most important story about it.
        </description>
        <icon type="local">assets/icon.png</icon>
        <launchable type="desktop-id">
            com.example.sampleapp.desktop
        </launchable>
        <releases>
            <release date="2019-11-27" version="4.2.8.0"/>
        </releases>
        <update_contact>example@example.com</update_contact>
        <url type="homepage">example.com</url>
        <url type="bugtracker">example.com</url>
        <url type="vcs-browser">example.com</url>
        <url type="translate">example.com</url>
        <url type="donation">example.com</url>
    </component>

A project file can adopt such information with the ``adopt-info`` key. Here's an example
configuration:

.. code-block:: yaml
    :caption: snapcraft.yaml with adopt-info

    name: sampleapp-name
    adopt-info: sampleapp

    apps:
      sampleapp:
        command: sampleapp
        common-id: com.example.sampleapp

    parts:
      sampleapp:
        plugin: dump
        source: http://github.com/example/sampleapp.git
        parse-info: [usr/share/metainfo/com.example.sampleapp.appdata.xml]

The path in ``parse-info`` is a relative path from the :ref:`part source, build, or
install directories <explanation-parts-lifecycle-directories>` (``CRAFT_PART_SRC``,
``CRAFT_PART_BUILD``, ``CRAFT_PART_INSTALL``).

The resulting snap will set the ``title``, ``version``, ``summary``, ``description``, ``license``, ``contact``,
``donation``, ``issues``, ``source-code`` and ``website`` from the AppStream file.

You can also link each app in your snap to specific AppStream metadata by pointing the
``common-id`` key of that app to the ``component id`` tag in the AppStream metadata.
Snapcraft will use the metadata of that component to get the ``.desktop`` entry file
for that app.

For backwards compatibility, some component ids in the AppStream metadata have a
``.desktop`` suffix. If this is the case for your application, the ``common-id`` of
your app should also use that suffix.

.. note::
    The process to get the ``.desktop`` file entry from the AppStream metadata goes as
    follows. First, Snapcraft searches for a parsed AppStream file with the same
    component id as the app's ``common-id`` and extracts the `Desktop File ID`_
    (``desktop-id``) from that component. If that component doesn't specify a
    ``desktop-id``, Snapcraft will use the component id as the Desktop File ID.
    Snapcraft will then search for a desktop file matching the Desktop File ID in the
    :file:`usr/local/share` and :file:`usr/share` directories relative to the part
    source, and by following the Desktop File ID rules.


.. _reference-external-package-scriptlets:

Part Scriptlets
~~~~~~~~~~~~~~~

Individual parts in your project file can set the ``version`` and ``grade`` keys by
using ``craftctl``. All you need to do is select which part to adopt using
``adopt-info``:

.. code-block:: yaml
    :caption: snapcraft.yaml with scriptlet metadata

    # ...
    adopt-info: my-part
    # ...
    parts:
      my-part:
        # ...
        override-pull:
          craftctl default
          craftctl set version="my-version"
          craftctl set grade="devel"

See :ref:`how-to-customize-the-build-and-part-variables` for more
details on using scripting elements within a project file.

.. _Desktop File ID: https://specifications.freedesktop.org/desktop-entry-spec/desktop-entry-spec-latest.html#desktop-file-id
