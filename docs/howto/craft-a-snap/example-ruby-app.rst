.. _example-ruby-app:

Example Ruby app
==================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a Ruby-based snap. We'll work through the aspects
unique to Ruby apps by examining an existing recipe.

The process of developing a snap for a Python app builds on top of YARV and
``gemspec`` file configuration, making it possible to adapt or integrate an
app's existing build tooling into the crafting process.


Example recipe for liquidctl
----------------------------

The following code comprises the recipe of a Ruby project, `Markdown lint tool
<https://github.com/snapcraft-docs/mdl>`_. This project provides style and
syntax checks for Markdown files.

.. collapse:: Markdown lint tool recipe

  .. code:: yaml

    name: test-mdl
    version: "0.5.0"
    summary: Markdown lint tool
    description: |
        Style checker/lint tool for markdown files.

    confinement: devmode
    base: core18

    parts:
        test-mdl:
            source: .
            plugin: ruby
            gems:
                - rake
                - bundler
            override-build: |
                snapcraftctl build
                rake install
            build-packages:
                - git

    apps:
        test-mdl:
          command: bin/mdl


Add a part written in Ruby
--------------------------

.. code:: yaml

  parts:
      test-mdl:
          source: .
          plugin: ruby
          gems:
              - rake
              - bundler
          override-build: |
              snapcraftctl build
              rake install
          build-packages:
              - git

Ruby parts are built with the `Ruby <https://snapcraft.io/docs/ruby-plugin>`_
plugin.

To declare a Ruby part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin`` to ``ruby``.
#. For the ``gems`` key, list any gem dependencies.
#. If you need the latest version of the gem bundler, set ``use-bundler`` to
   ``true``.

