.. _how-to-craft-a-ruby-app:

Craft a Ruby app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a Ruby-based snap. We'll work through the aspects
unique to Ruby apps by examining an existing project.

The process of developing a snap for a Python app builds on top of YARV and
``gemspec`` file configuration, making it possible to adapt or integrate an
app's existing build tooling into the crafting process.


Example project file for Markdown lint tool
-------------------------------------------

The following code comprises the project file of a Ruby app, the `Markdown lint tool
<https://github.com/snapcraft-docs/mdl>`_. This project provides style and syntax checks
for Markdown files.

.. collapse:: Markdown lint tool project file

    .. literalinclude:: ../code/integrations/example-ruby-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add a part written in Ruby
--------------------------

.. literalinclude:: ../code/integrations/example-ruby-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: parts:
    :end-at: - git

Ruby parts are built with the `Ruby <https://snapcraft.io/docs/ruby-plugin>`_
plugin.

To declare a Ruby part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``build-packages``, and so on.
#. Set ``plugin: ruby``.
#. For the ``gems`` key, list any gem dependencies.
#. If you need the latest version of the gem bundler, set ``use-bundler`` to
   ``true``.
