.. _explanation-components:

Components
**********

.. include:: /reuse/components-intro.rst

Components are useful for distributing optional resources alongside a snap. For
example, debug symbols are useful for the developer of an application and are
closely linked to a particular build of an application. Debug symbols are not
useful for the users. Including debug symbols in a snap package would waste
user's network bandwidth and disk space. If an application packages debug
symbols as a component alongside the snap, then the developers who require
debug symbols can download and install them as a component.

.. note::

   Components are under development and not production ready.

.. _components-and-partitions:

Components and Partitions
-------------------------

Components utilise a `Craft Parts`_ feature called ``partitions``. This feature
is enabled only when the ``component`` key is defined in the project file.

Each component has a namespaced partition ``component/<component-name>`` where
``component`` is the partition's namespace and ``<component-name>`` is the name
of the component from the project file.

The partition for the snap itself is known as the ``default`` partition.

Component lifecycle directories
-------------------------------

When a part is built, the output is the default partition's install directory
for that part.

Each component has a partition. This means each component has its own install,
stage, and prime directories. When a file is organized into a component's
partition, it is moved to the part's install directory for that component's
partition.

For example, consider a part with the ``organize`` key:

.. code-block:: yaml
    :caption: snapcraft.yaml

    parts:
      my-part:
        plugin: nil
        override-build: |
          touch $CRAFT_PART_INSTALL/hello
        organize:
          hello: (component/translations)/hello-world

This part's build creates a file called ``hello`` in ``my-part``'s default
install directory. If no ``organize`` key was used, this file would be
included in the snap itself.

However this example uses the ``organize`` key to move the file ``hello``
from ``my-part``'s default install directory to ``my-parts``'s install
directory for the ``translations`` component. It also renames the file from
``hello`` to ``hello-world``.

The packed snap will contain nothing from this part and the ``translations``
component will contain the ``hello-world`` file.
