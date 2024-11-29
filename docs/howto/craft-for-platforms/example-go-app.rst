.. _example-go-app:

Example Go app
==============

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a Go-based snap. We'll work through the aspects
unique to Go apps by examining an existing recipe.

The process of developing a snap for a Go app builds on top of the standard Go
compiler and configuration, making it possible to adapt or integrate an app's
existing build tooling into the crafting process.


Example recipe for woke
-----------------------

The following code comprises the recipe of a Go project, `woke
<https://github.com/get-woke/woke>`_. This project is a text analysis tool that
detects exclusive language.

.. collapse:: woke recipe

  .. code:: yaml

    name: woke
    summary: Detect non-inclusive language in your source code
    description: |
        Creating an inclusive work environment is imperative to a healthy,
        supportive, and productive culture, and an environment where everyone
        feels welcome and included. woke is a text file analysis tool that finds
        places within your source code that contain non-inclusive language and
        suggests replacing them with more inclusive alternatives.
    adopt-info: woke
    base: core22

    confinement: devmode

    plugs:
        dot-config-woke:
            interface: personal-files
            read:
                - $HOME/.config/woke.yaml
                - $HOME/.config/woke.yml
                - $HOME/.woke.yaml
                - $HOME/.woke.yml

    apps:
        woke:
            command: bin/woke
            plugs:
                - home
                - dot-config-woke
                - network
                - removable-media

    parts:
        woke:
            plugin: go
            build-snaps: [go/latest/stable]
            source: https://github.com/get-woke/woke
            source-type: git
            override-pull: |
                snapcraftctl pull
                snapcraftctl set-version \
                "$(git describe --long --tags --always --match=v*.*.* | sed 's/v//')"


Add a part written in Go
------------------------

.. code:: yaml

  parts:
      woke:
          plugin: go
          build-snaps: [go/latest/stable]
          source: https://github.com/get-woke/woke
          source-type: git
          override-pull: |
              snapcraftctl pull
              snapcraftctl set-version \
              "$(git describe --long --tags --always --match=v*.*.* | sed 's/v//')"

Go parts are built with the `Go plugin <https://snapcraft.io/docs/go-plugin>`_.

To declare a Go part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: go``.
#. If necessary, you can override the Go compiler version by listing it in the
   ``build-snaps`` key, in the format ``go/<track>/<risk>``. The
   latest version is ``go/latest/stable``.
