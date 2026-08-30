.. _how-to-craft-a-java-app:

Craft a Java app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a Java-based snap. We'll work through the aspects
unique to Java apps by examining an existing project.

The process of developing a snap for a Java app builds on top of JRE, making it
possible to adapt and integrate an app's existing build tooling and
dependencies into the crafting process.


Example project file for Cal
----------------------------

The following code comprises the project file of a Java tool, `cal
<https://github.com/frossm/cal>`_. This project is both a driver and a CLI tool for
power and cooling components in PCs.

.. dropdown:: Cal project file

    .. literalinclude:: ../code/integrations/example-java-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add a part written in Java
--------------------------

.. literalinclude:: ../code/integrations/example-java-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :dedent: 2
    :lines: 50-

Java parts are built with the :ref:`Maven plugin <maven_plugin>`. The plugin
can build the app using standard parameters. It requires a ``pom.xml``
file at the root of the source tree.

To declare a Java part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``organize``, and so on.
#. Set ``plugin: maven``.
#. For ``build-packages``, list the following dependencies:

   .. literalinclude:: ../code/integrations/example-java-recipe.yaml
       :caption: snapcraft.yaml
       :language: yaml
       :start-at: - maven
       :dedent: 6
       :end-at: - openjdk-11-jdk-headless

#. For ``stage-packages``, add ``openjdk-11-jre-headless`` as a dependency.
