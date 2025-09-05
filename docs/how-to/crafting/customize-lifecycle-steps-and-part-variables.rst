.. |app| replace:: Snapcraft
.. |artifact| replace:: snap
.. |artifact-indefinite| replace:: a snap

.. _how-to-customize-the-build-and-part-variables:

Customize lifecycle steps and project variables
===============================================

.. include:: /common/craft-parts/how-to/customise-the-build-with-craftctl.rst
    :start-line: 7
    :end-before: For example, if you want to set the ``version`` key of |artifact-indefinite| to the

The ``version`` and ``grade`` keys are supported.

To incorporate retrieved package information correctly, point ``adopt-info`` to the part
that runs ``craftctl set``. Without such a connection, the snap fails to build. You can
find more guidance on sourcing information in
:ref:`reference-external-package-information`.


Example solutions
~~~~~~~~~~~~~~~~~

.. include:: /common/craft-parts/how-to/customise-the-build-with-craftctl.rst
    :start-after: craftctl get <key>
    :end-before: Expose part variables in apps

.. _how-to-access-project-variables-across-parts-and-components:

Access project variables across parts and components
----------------------------------------------------

You can further use craftctl to get and set custom variables within different
components using a combination of the ``adopt-info`` tag and a selector syntax. For example:

.. code-block:: yaml
    :caption: Project file

    version: "1.0"
    adopt-info: my-library

    components:
      debug-symbols:
        type: standard
        adopt-info: my-binary

    parts:
      my-library:
        source: ./lib
        plugin: dump
        override-pull: |
          craftctl set version="$(cat VERSION.txt)"
          craftctl default
        override-build: |
          craftctl get version

      my-binary:
        source: ./src
        plugin: dump
        override-pull: |
          craftctl set components.debug-symbols.version="$(cat VERSION.txt)"
          craftctl default

The ``components.debug-symbols.version`` selector picks up the component's version,
which is distinct from the ``version`` variable set in the ``my-library`` part.

Note also how we use ``adopt-info`` at both the top level, and within a component.
In this example, the project variables set in the ``my-binary`` part will be included
in the scope of the ``debug-symbols`` component.
