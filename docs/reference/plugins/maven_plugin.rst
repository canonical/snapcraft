.. _maven_plugin:

.. include:: /common/craft-parts/reference/plugins/maven_plugin.rst
   :end-before: .. _maven-details-begin:

Dependencies
------------

The plugin expects Maven to be available on the system as the ``mvn``
executable, unless a part named ``maven-deps`` is defined. In this
case, the plugin will assume that this part will stage the ``mvn``
executable to be used in the build step.

Note that the Maven plugin does not make a Java runtime available in
the target environment. In Snapcraft, the developer must stage the
appropriate packages in order to be able to execute the application
(unless the Java runtime is provided by a
content snap).


Example
-------

This is an example of a Snapcraft part using the Maven plugin. Note
that the Maven and Java Runtime packages are listed as build packages,
and the Java Runtime is staged to be part of the final payload::

  mkpass:
    plugin: maven
    source: .
    build-packages:
      - openjdk-11-jre-headless
      - maven
    stage-packages:
      - openjdk-11-jre-headless

.. include:: /common/craft-parts/reference/plugins/maven_plugin.rst
   :start-after: .. _maven-details-end:
