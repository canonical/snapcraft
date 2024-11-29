.. _example-java-app:

Example Java app
================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a Java-based snap. We'll work through the aspects
unique to Java apps by examining an existing recipe.

The process of developing a snap for a Java app builds on top of JRE, making it
possible to adapt and integrate an app's existing build tooling and
dependencies into the crafting process.


Example recipe for Cal
----------------------

The following code comprises the recipe of a Java project, `liquidctl
<https://github.com/frossm/cal>`_. This project is both a driver and a CLI
tool for power and cooling components in PCs.

.. collapse:: Cal recipe

  .. code:: yaml

    name: fcal
    version: '2.7.1'
    summary: Command line calendar display
    description: |
        fCal is a command line calendar utility. It will display a calendar on
        the command line with any month/year requested. Defaults to the current
        year. fCal can also display local holidays. See help.
    grade: stable
    confinement: strict
    base: core22

    title: fCal
    website: https://github.com/frossm/cal/issues
    issues: https://github.com/frossm/ca
    license: MIT

    # Enable faster LZO compression
    compression: lzo

    # Ignore useless library warnings
    lint:
        ignore:
          - library

    apps:
        fcal:
          command: cal-wrapper
          plugs:
            - network

    parts:
        wrapper:
            plugin: dump
            source: snap/local
            source-type: local

        library:
            plugin: maven
            source: https://github.com/frossm/library.git
            source-type: git
            source-tag: 'v2023.12.03'
            maven-parameters:
                - install
            build-packages:
                - maven
                - openjdk-11-jdk-headless

        cal:
            plugin: maven
            source: https://github.com/frossm/cal.git
            source-branch: master
            source-type: git
            after:
                - library
            build-packages:
                - maven
                - openjdk-11-jdk-headless
            stage-packages:
                - openjdk-11-jre-headless
            override-prime: |
                snapcraftctl prime
                rm -vf usr/lib/jvm/java-11-openjdk-*/lib/security/blacklisted.certs


Add a part written in Java
--------------------------

.. code:: yaml

  cal:
    plugin: maven
    source: https://github.com/frossm/cal.git
    source-branch: master
    source-type: git
    after:
        - library
    build-packages:
        - maven
        - openjdk-11-jdk-headless
    stage-packages:
        - openjdk-11-jre-headless
    override-prime: |
        snapcraftctl prime
        rm -vf usr/lib/jvm/java-11-openjdk-*/lib/security/blacklisted.certs

Java parts are built with the :ref:`Maven plugin <maven_plugin>`. The plugin
can build the app using standard parameters. It requires a ``pom.xml``
file at the root of the source tree.

To declare a Java part:

#. Declare the general part keys, such as ``source``, ``override-build``,
   ``organize``, and so on.
#. Set ``plugin: maven``.
#. For ``build-packages``, list the following dependencies:

   .. code:: yaml

     - maven
     - openjdk-11-jdk-headless

#. For ``stage-packages``, add ``openjdk-11-jre-headless`` as a dependency.
