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
