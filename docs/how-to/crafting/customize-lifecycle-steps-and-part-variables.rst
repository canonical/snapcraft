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


Accessing project variables across parts and components
-------------------------------------------------------

You can further use craftctl to get and set custom variables within different
components using a combination of the ``adopt-info`` tag and a dot syntax. For example:

.. code-block:: yaml
    :caption: Project file

    version: "1.0"
    adopt-info: part-1

    components:
      component-1:
        type: test
        adopt-info: part-2

    parts:
      part-1:
        source: src-1
        plugin: dump
        override-pull: |
          craftctl set version="$(cat VERSION.txt)"
          craftctl default
        override-build: |
          craftctl get version

      part-2:
        source: src-2
        plugin: dump
        override-pull: |
          craftctl set components.component-1.version="$(cat VERSION.txt)"
          craftctl default

Note the syntax ``components.component-1.version``, which is a distinct value from
the ``version`` variable referenced in ``part-1``.

Note also how we use ``adopt-info`` at both the top level, and within a component.
In this example, the project variables set in ``part-2`` will be included in the
scope of ``component-1``.
