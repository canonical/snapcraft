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


Craft a snap from Electron source
----------------------------------

For apps that are open source, you can build the Electron application directly
from its source repository instead of repackaging a pre-built binary. This
approach is more transparent and reproducible.


Example Signal Desktop project file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following code comprises the project file of `Signal Desktop
<https://github.com/snapcrafters/signal-desktop>`_, a privacy-focused messaging
application built with Electron. It fetches the upstream source and builds
using ``pnpm``, demonstrating how to craft a snap from an Electron source tree.

.. dropdown:: Signal Desktop project file

    .. code-block:: yaml
        :caption: snapcraft.yaml

        name: signal-desktop
        title: Signal Desktop
        summary: Speak Freely - Private Messenger
        description: |
          Millions of people use Signal every day for free and instantaneous
          communication anywhere in the world.
        website: https://signal.org/
        source-code: https://github.com/snapcrafters/signal-desktop
        license: AGPL-3.0-only
        icon: snap/gui/signal-desktop.png
        version: 8.31.0

        base: core24
        grade: stable
        confinement: strict
        compression: lzo

        platforms:
          amd64:
          arm64:

        parts:
          node-runtime:
            plugin: nil
            build-packages:
              - curl
            override-build: |
              # Install pnpm in a dedicated part so Signal's workspace hooks
              # are not loaded while bootstrapping pnpm itself.
              export PNPM_HOME="${CRAFT_PART_INSTALL}/pnpm"
              export PATH="$PNPM_HOME/bin:$PNPM_HOME:$PATH"
              curl -fsSL https://get.pnpm.io/install.sh -o /tmp/install-pnpm.sh
              (cd /tmp && env SHELL=bash sh /tmp/install-pnpm.sh)
            prime:
              - -*

          signal-desktop:
            after:
              - node-runtime
            plugin: dump
            source: https://github.com/signalapp/Signal-Desktop
            source-type: git
            source-tag: v$SNAPCRAFT_PROJECT_VERSION
            build-packages:
              - jq
              - python3
            build-environment:
              - SIGNAL_ENV: "production"
            override-build: |
              export PNPM_HOME="${CRAFT_STAGE}/pnpm"
              export PATH="$PNPM_HOME/bin:$PNPM_HOME:$PATH"
              pnpm env use --global "$(jq -r .engines.node package.json)"
              pnpm install --frozen-lockfile
              pnpm run build:release
              craftctl default


Electron parts
~~~~~~~~~~~~~~

In the ``signal-desktop`` part above, the ``dump`` plugin is used together
with ``override-build`` to compile the application via ``pnpm`` before staging
the output. The ``node-runtime`` part installs the ``pnpm`` package manager in
isolation so that Signal's workspace hooks do not interfere with the
bootstrapping step.

This pattern — pulling source from the upstream Git repository, installing
Node.js at the required version, and running the project's own build script —
is the recommended approach when you want a reproducible, auditable build
rather than repackaging a pre-built binary.
