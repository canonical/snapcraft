.. _reference_metadata:

Using external metadata
=======================

To help avoid unnecessary duplication, and for convenience, Snapcraft can process and
incorporate external metadata from within a project file by using the ``parse-info``
key within a :ref:`part <parts>` and a corresponding ``adopt-info`` key.

For example, the following project file will parse a file called ``metadata-file``.
Snapcraft will attempt to extract ``version``, ``summary``, and ``description``
metadata for the snap, all of which are mandatory:

.. code-block:: yaml
    :caption: metadata in snapcraft.yaml

    name: my-snap-name
    adopt-info: part-with-metadata

    parts:
      part-with-metadata:
        plugin: dump
        source: .
        parse-info: [metadata-file]

See `The Snapcraft format <https://snapcraft.io/docs/build-configuration>`_ for further
details on Snapcraft metadata and how it's used.

Source Types
------------

An external metadata source can be one of the following:

- :ref:`AppStream <reference_metadata-appstream>`: a standard for software
  components
- :ref:`Scriptlets <reference_metadata-scriptlets>`: a snapcraftctl-driven command to
  generate ``version`` and ``grade``.

.. _reference_metadata-appstream:

AppStream
~~~~~~~~~

`AppStream`_ is a metadata standard used to describe a common set of software
components. It can be parsed by Snapcraft to provide the ``title``, ``version``,
``summary``, ``description``, and ``icon`` keys for a snap, along with the location of
an app's `desktop <https://snapcraft.io/docs/desktop-menu-support>`_ file.

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

We *adopt* the above metadata into the project file with the following:

.. code-block:: yaml
    :caption: snapcraft.yaml with metadata adoption

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

.. note::
    The path in ``parse-info`` is a relative pasth from the part source, build or
    install directory (`CRAFT_PART_SRC, CRAFT_PART_BUILD, CRAFT_PART_INSTALL
    <https://snapcraft.io/docs/parts-lifecycle#heading--parts-directories>`_)

The resulting snap will use the title, version, summary, description, license, contact,
donation, issues, source-code and website from the AppStream file.

You can also link each app in your snap to specific AppStream metadata by pointing the
``common-id`` key of that app to the ``component id`` field in the AppStream metadata.
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

.. _reference_metadata-scriptlets:

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

See `Using the craftctl tool <https://snapcraft.io/docs/using-craftctl>`_ for more
details on using scripting elements within a project file.

[not recommended] ``setup.py``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. warning::
    Using ``parse-info`` with :file:`setup.py` is currently discouraged because it has
    many issues. For example, it incorrectly `uses the project's summary as the snap's
    description <https://bugs.launchpad.net/snapcraft/+bug/1813364>`_ and it `might
    crash the snap build
    <https://github.com/snapcore/snapcraft/pull/2756#issuecomment-544284814>`_.

A `setup.py <https://docs.python.org/3/distutils/setupscript.html>`_ file is used by
many Python projects to help with package installation. If your :file:`setup.py` uses
`setuptools <https://setuptools.readthedocs.io/en/latest/>`_ and defines ``version``
and ``description``, these can be extracted from :file:`setup.py` and used as the
``version`` and ``description`` metadata in the resulting snap.

The following is an example of :file:`setup.py` in the root of a hypothetical git tree:

.. code-block:: python
    :caption: a basic example setup.py

    import setuptools

    setuptools.setup(
        name='hello-world',
        version='1.0',
        author='snapcrafter',
        author_email='snapcraft@lists.snapcraft.io',
        description='A simple hello world in python',
        scripts=['hello'],
    )

You can *adopt* the relevant metadata in the baove with the following project file:

.. code-block:: yaml
    :caption: snapcraft.yaml adopting from setup.py

    name: sampleapp-name
    summary: sampleapp summary
    adopt-info: sampleapp

    apps:
      sampleapp:
        command: sampleapp

    parts:
      sampleapp:
        plugin: python
        source: https://github.com/example/sampleapp.git
        parse-info: [setup.py]

Snapcraft versions and compatibility
------------------------------------

.. list-table::
    :header-rows: 1

    * - Change
      - Snapcraft version

    * - Initial introduction
      - 2.39

    * - appstream support
      - 2.39

    * - ``common-id``
      - 2.40

    * - :file:`setup.py` support
      - 2.41

    * - ``snapcraftctl set-version``
      - 2.41

    * - ``snapcraftctl set-grade``
      - 2.41

.. _Desktop File ID: https://specifications.freedesktop.org/desktop-entry-spec/desktop-entry-spec-latest.html#desktop-file-id
