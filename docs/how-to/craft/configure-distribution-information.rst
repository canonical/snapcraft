.. _how-to-configure-distribution-information:

Configure distribution information
==================================

Many of the basic properties in a snap's project file define information for the snap's
distribution, such as identity, authors, version, desktop data, and store data. This
information is also known as *metadata* and the snap's *manifest*.

Distribution information is a mixture of required and optional keys. This guide covers
the required information first. For the non-essential keys, review the Top-level
directives in the `snapcraft.yaml reference
<https://snapcraft.io/docs/snapcraft-yaml-schema#p-21225-top-level-directives>`_.


Configure the required information
----------------------------------

When you initialize a snap, it populates ``snapcraft.yaml`` with the minimal set of
required keys.

The placeholder values looks similar to this:

.. code-block:: yaml
    :caption: snapcraft.yaml

    name: newtest
    base: core24
    version: '0.1'
    summary: Single-line elevator pitch for your amazing snap
    description: |
      This is my-snap's description. You have a paragraph or two to tell the
      most important story about your snap. Keep it under 100 words though,
      we live in tweetspace and your description wants to look good in the snap
      store.

    grade: devel
    confinement: devmode

    parts:
      my-part:
        plugin: nil

When crafting a snap, fill these keys as follows:

.. For help on choosing a name and registering it on the Snap Store, see `Registering your app name <>`_.

.. list-table::

    * - Key
      - Action
    * - ``name``
      - Give your snap a unique name. The name can only contain lowercase letters,
        numbers, and hyphens. It must start with an ASCII character, and can't end with
        a hyphen. If you plan on publishing your snap, it must be globally unique among
        all public and private stores.
    * - ``base``
      - Set the version of Ubuntu that the snap will use for its runtime environment.
        :ref:`how-to-bases` are a complex topic that is out of scope for this guide.
        Unless you're building a snap compatible with older code, leave this as
        ``core24``.
    * - ``version``
      - Set the initial version of your snap. This key is a simple string, so you can
        use any version schema. You can later replace this with a different version, or
        fill this string automatically with a script.
    * - ``summary``
      - Provide a short sentence to tell prospective users about your snap's purpose. It
        must be in fewer than 80 characters.
    * - ``description``
      - Describe your snap in as much detail and space as you need. Notice the pipe (|)
        on the first line, which splits the description across multiple lines. The text
        is processed as Markdown, so most Markdown syntax is supported.

        You should keep the length reasonable, but the more details you provide, the
        more likely people are to discover and use your snap. Feature lists, update
        descriptions, and a brief *Getting started* guide are legitimate uses for the
        description.
    * - ``grade``
      - Specify the production readiness of your snap. While developing, leave this set
        to ``devel`` to disable the snapd guardrails. When your snap is ready for
        release, set it to ``stable``.
    * - ``confinement``
      - Set how strong the sandboxing of the snap is. A snap's confinement level is the
        degree of isolation it has from the host system. When first crafting, leave this
        as ``devmode`` to disable sandboxing until you have a working snap. In the rare
        case where your snap needs higher levels of system access, like a traditional
        unsandboxed package, you can :ref:`enable classic confinement
        <how-to-enable-classic-confinement>`.


Reuse information
-----------------

When you're crafting a snap for existing software, it's easier to import its
distribution information from an external source instead of manually copying it. There
are three sets of data available. You can:

- Copy the main distribution information from a standard `AppStream`_ metadata file.
- Copy the app's existing ``.desktop`` file by reading its component ID from the
  AppStream metadata.
- Set the snap's version and grade through a script.


.. _how-to-configure-distribution-information-appstream-metadata:

Reuse the AppStream package information
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Snapcraft can parse AppsStream metadata files to provide the title, version, summary,
description, and icon for a snap, along with the location of an app's ``.desktop`` file.

The following sample is a typical AppStream metadata file for a software project.

.. code-block:: xml
    :caption: bravo.metainfo.xml

    <?xml version="1.0" encoding="UTF-8"?>
    <component type="desktop-app">
      <id>com.alex.bravo</id>
      <name>Bravo</name>
      <project_license>GPL-3.0+</project_license>
      <summary>Single-line elevator pitch for your amazing app</summary>
      <description>
        This is apps's description. A paragraph or two to tell the most
        important story about it.
      </description>
      <icon type="local">assets/icon.png</icon>
      <launchable type="desktop-id">com.alex.bravo.desktop</launchable>
      <releases>
          <release date="2025-01-01" version="1.5.3"/>
      </releases>
      <update_contact>alex@example.com</update_contact>
      <url type="homepage">https://example.com</url>
      <url type="bugtracker">https://example.com/issues</url>
      <url type="vcs-browser">https://github.com/alex/bravo</url>
      <url type="translate">https://example.com</url>
      <url type="donation">https://example.com/donate</url>
    </component>

If you were packaging this project as a snap, you're better served by copying this
distribution data rather than replicating it manually.

The keys that copy this information are ``adopt-info`` at the start of the project file
and ``parse-info`` in the definition for the main part -- typically the main app.

To copy the info, first remove the distribution keys that you're replacing with the
metadata file. These could be, among others, ``title``, ``description``, ``summary``,
``license``, ``contact``, ``donation``, ``issues``, ``source-code``, ``license``, and
``website``.

Then, set ``adopt-info`` to the name of part that contains the metadata file.

Lastly, in the main part definition, set ``parse-info`` to the path of the metadata
file. The path is relative to one of the part's internal directories in the snap
filesystem, being one of ``source`` (``CRAFT_PART_SRC``), ``build``
(``CRAFT_PART_BUILD``), or ``install`` (``CRAFT_PART_INSTALL``).

During build, Snapcraft will now reuse all compatible distribution information from the
metadata file.

This setup is demonstrated here:

.. code-block:: yaml
    :caption: snapcraft.yaml
    :emphasize-lines: 2, 13

    name: bravo
    adopt-info: bravo-part

    apps:
      bravo:
        command: bravo

    parts:
      bravo-part:
        plugin: dump
        source: http://github.com/alex/bravo.git
        parse-info:
          - usr/share/metainfo/com.alex.bravo.appdata.xml


Copy the ``.desktop`` file from AppStream
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For backward compatibility, component IDs in the AppStream metadata can have a
``.desktop`` suffix. If this is the case for the app you're packaging, you can reuse the
file by sourcing it with a special key in the app's definition in your project file.

First, find the component ID in the metadata file. It should be in a ``launchable`` tag,
contain the same ID as the app itself, and end in ``.desktop``. In our metadata file
example, it was:

.. code-block:: xml
    :caption: sampleapp.metainfo.xml

    <launchable type="desktop-id">com.example.sampleapp.desktop</launchable>

Then, in the app's definition in the project file, set the ``common-id`` key to the
app's component ID, *without* the ``.desktop`` extension.

During build, Snapcraft will now copy the ``.desktop`` file into the app from the part.

See this configuration here:

.. code-block:: yaml
    :caption: snapcraft.yaml
    :emphasize-lines: 7

    name: bravo
    adopt-info: bravo-part

    apps:
      bravo:
        command: bravo
        common-id: com.alex.bravo

    parts:
      bravo-part:
        plugin: dump
        source: http://github.com/alex/bravo.git
        parse-info:
          - usr/share/metainfo/com.alex.bravo.appdata.xml


From a script in a part
~~~~~~~~~~~~~~~~~~~~~~~

If you need to procedurally define the snap's ``version`` and ``grade`` keys, you can
set them at build time with the ``craftctl`` command, invoked by a script.

Start by setting ``adopt-info`` to the name of an important part, typically the snap's
main part.

After that, set ``override-pull`` to an inline series of ``craftctl`` commands. The
variables ``version`` and ``grade`` map to the keys with the same names. You can set
them like environment variables with the ``set`` verb. Here, use any external source you
prefer that's accessible through commands in the host environment, such as environment
variables or an API endpoint.

During build, Snapcraft will set the snap's version and grade based on the values
from the source you provided.

Here's an example of that configuration:

.. code-block:: yaml
    :caption: snapcraft.yaml
    :emphasize-lines: 2, 9-12

    name: bravo
    adopt-info: bravo-part
    ...

    parts:
      bravo-part:
        plugin: dump
        source: http://github.com/alex/bravo.git
        override-pull: |
          craftctl default
          craftctl set version="1.5.3"
          craftctl set grade="stable"


Configure the desktop entry
---------------------------

Snaps support the Linux `desktop entry
<https://specifications.freedesktop.org/desktop-entry-spec/latest>`_ standard. You can
use desktop entry files to define your snap's entry in the desktop environment's various
app menus and launchers. The file controls the entry's presentation and how it launches.
If configured, snapd will automatically add your snap to the app launcher and menus
during installation.

There are three methods to provide the desktop menu entry:

- Copy the desktop entry file from the app's files.
- Add the desktop entry file to the snap.
- `Copy the desktop entry file
  <how-to-configure-distribution-information-appstream-metadata>`_ through the app's
  AppStream metadata.

.. important::

    The icon in the desktop entry is separate from the ``icon`` key. The latter is used
    in graphical front ends, like the snap's profile in the Snap Store.


Read a desktop entry file
~~~~~~~~~~~~~~~~~~~~~~~~~

Some apps generate desktop files as part of the build process. If that's the case, it's
easier to read the desktop entry file already in the app.

First, in the main app's definition, set the ``desktop`` key to the path of the
``.desktop`` file. The key accepts a path relative to the ``prime`` directory during the
prime step of the build, so it must match the file's location during that step.

Still in the main app, connect the `desktop interface
<https://snapcraft.io/docs/desktop-interface>`_.

Lastly, make sure that the ``Icon`` path in the desktop entry is accessible from the
``prime`` folder. During build, Snapcraft will attempt to automatically resolve the
``Icon`` path. If the final path is incorrect, correct it by adding the
``override-pull`` key on the main part and listing commands that would correct the path.

In the following example, the desktop file is generated by the build system and placed
in the ``usr/share/apps/`` directory at the root of the snap filesystem. It specifies
``usr/share/apps/com.alex.bravo.desktop`` as the path to the desktop file. During the
pull step, it corrects the ``Icon`` path in the desktop entry with ``override-pull``.

.. code-block:: yaml
    :caption: snapcraft.yaml

    apps:
      bravo:
        command: desktop-launch $SNAP/usr/bin/com.alex.bravo
        desktop: usr/share/apps/com.alex.bravo.desktop
        plugs:
          - desktop
          - desktop-legacy

    parts:
      bravo:
        plugin: meson
        meson-parameters: [--prefix=/snap/bravo/current/usr]
        override-pull: |
          snapcraftctl pull

          # Point icon to the correct location
          sed -i.bak -e \
          's|Icon=com.alex.bravo|Icon=/usr/share/icons/hicolor/scalable/apps/com.alex.bravo.svg|g' \
          data/com.alex.bravo.desktop.in


Add a desktop entry file
~~~~~~~~~~~~~~~~~~~~~~~~

With this basic method, you manually add a desktop entry file to the snap.

To start, create files named ``<snap-name>.desktop`` and ``<snap-name>.png`` in the
``snap/gui/`` directory in your project's source. Replace ``<snap-name>`` with the same
value as the ``name`` key in the project file.

For the desktop entry file, enter:

.. code-block:: desktop
    :caption: .desktop file

    [Desktop Entry]
    Exec=<app-name>
    Icon=${SNAP}/meta/gui/<snap-name>.png

Replace ``<app-name>`` with the same name you gave the app in the project file. The
name is case-sensitive.

Assign ``Icon`` to the absolute path of the image file. This path must be the location
of the icon after the snap is installed. Since snapcraft copies all the contents of the
``snap/gui/`` folder to ``meta/gui`` during installation, the absolute path of the icon
in this arrangement is ``${SNAP}/meta/gui/snapname.png``.
