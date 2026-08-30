.. _how-to-craft-an-electron-app:

Craft an Electron app
=====================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap of an app built using Electron. We'll work
through the aspects unique to Electron-based apps by examining an existing
project file.

Electron supports snaps as part of its out-of-the-box build system. You can
extend it to automatically create a snap, or wrap an existing binary in a
separate snap project file.


Craft a snap with the Electron build tools
------------------------------------------

To craft with the built-in tooling, you only need to modify the project's
``package.json`` manifest.


Requirements
~~~~~~~~~~~~

For built-in snap support, you need:

- Electron 9 or higher
- electron-builder 22.7.0 or higher


Example Electron package manifest
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following code comprises the ``package.json`` file of the starter Electron
project, `electron-quick-start
<https://github.com/electron/electron-quick-start>`_, but modified to build
snaps.

.. dropdown:: electron-quick-start package manifest

  .. code:: json

    {
      "name": "electron-quick-start",
      "version": "1.0.0",
      "description": "A minimal Electron application",
      "main": "main.js",
      "scripts": {
        "start": "electron .",
        "dist": "electron-builder --linux snap"
      },
      "repository": "https://github.com/electron/electron-quick-start",
      "keywords": [
        "Electron",
        "quick",
        "start",
        "tutorial",
        "demo"
      ],
      "author": "GitHub",
      "license": "CC0-1.0",
      "devDependencies": {
        "electron": "^11.2.1",
        "electron-builder": "^22.9.1"
      }
    }

To build an Electron-based snap with the built-in tooling:

#. For ``scripts``, add ``dist: electron-builder --linux snap``.
#. Add electron-builder as a dependency:

   - Let npm handle installation and manifest by running:

     .. code:: bash

       npm install --save-dev electron-builder

   - Alternatively, in ``package.json``, for ``devDependencies`` add
     ``electron-builder`` and set it to ``22.7.0`` or higher.
#. Build the snap:

   .. code:: bash

     npm run dist

The builder will generate a snap in the ``dist/`` directory.


Craft a snap of an existing binary
----------------------------------

Alternatively, if you have a pre-built binary such as a deb or tarball file,
you can craft that into a snap, too.


Example Discord project file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following code comprises the project file of an Electron app, `Discord
<https://github.com/snapcrafters/discord>`_. It's a popular instant messaging app and
social platform.

.. dropdown:: Discord project file

    .. code-block:: yaml
        :caption: snapcraft.yaml

        name: discord
        title: Discord
        summary: Chat for Communities and Friends
        description: |
          Discord is the easiest way to communicate over voice, video, and text.
          Chat, hang out, and stay close with your friends and communities.

          Snaps are confined, as such Discord may be unable to perform some of
          the tasks it typically does when unconfined. This may result in the
          system log getting spammed with apparmor errors. Granting access to the
          system-observe interface when in the snap will enable the features, and
          thus reduce the logging.

            snap connect discord:system-observe

          **Authors**

          This snap is maintained by the Snapcrafters community, and is not
          necessarily endorsed or officially maintained by the upstream
          developers.

        website: https://discord.com/
        contact: https://github.com//snapcrafters/discord/issues
        issues: https://github.com//snapcrafters/discord/issues
        source-code: https://github.com//snapcrafters/discord
        license: Proprietary
        icon: snap/discord.png
        version: 0.0.76

        base: core22 # Reverted to core22 as a temporary workaround for https://github.com/snapcrafters/discord/issues/233
        grade: stable
        confinement: strict
        compression: lzo

        assumes:
          - snapd2.54

        architectures:
          - amd64

        parts:
          launcher:
            plugin: dump
            source: snap/local
            source-type: local
            stage-packages:
              - jq

          discord:
            plugin: dump
            source: https://dl.discordapp.net/apps/linux/${SNAPCRAFT_PROJECT_VERSION}/discord-${SNAPCRAFT_PROJECT_VERSION}.deb
            source-type: deb
            override-build: |
              craftctl default
              sed -i 's|Icon=discord|Icon=/usr/share/discord/discord\.png|' ${CRAFT_PART_INSTALL}/usr/share/discord/discord.desktop
            stage-packages:
              - libatomic1
              - libc++1
              - libnspr4
              - libnss3
              - libxss1
              - xdg-utils
            prime:
              - -usr/share/discord/chrome-sandbox
              - -usr/bin/xdg-open

        plugs:
          shmem:
            interface: shared-memory
            private: true

        apps:
          discord:
            extensions: [gnome]
            command: bin/launcher
            command-chain: [bin/disable-updater]
            autostart: discord-stable.desktop
            desktop: usr/share/applications/discord.desktop
            environment:
              # Correct the TMPDIR path for Chromium Framework/Electron to
              # ensure libappindicator has readable resources
              TMPDIR: $XDG_RUNTIME_DIR
              DISABLE_WAYLAND: 1
              # Included temporarily until https://github.com/snapcore/snapcraft-desktop-integration/issues/28
              # is resolved.
              NOTIFY_IGNORE_PORTAL: 1
            plugs:
              - audio-playback
              - audio-record
              - camera
              - home
              - mount-observe
              - network
              - network-observe
              - process-control
              - removable-media
              - screen-inhibit-control
              - shmem
              - system-observe
              - unity7


Electron parts
~~~~~~~~~~~~~~

Since they are wrapped binaries, Electron parts don't have a custom plugin and instead
use the :ref:`craft_parts_dump_plugin`.

In the definition of the ``discord`` part, you can see that we set the source
to the official Debian archive published by the Discord authors, and then
remove ``chrome-sandbox``, as the browser-sandbox is unnecessary in simple
Electron apps.
