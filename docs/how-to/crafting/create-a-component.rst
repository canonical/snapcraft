.. _how-to-create-a-component:

Create a component
==================

.. include:: ../../reuse/components-intro.rst

Start with a simple project file:

.. literalinclude:: ../code/crafting/basic/snapcraft.yaml
    :caption: snapcraft.yaml
    :language: yaml

And create the following project tree:

.. code-block:: text

   .
   └── src
       └── my-app

``my-app`` can be an empty file. The contents are not important for this
how-to guide.

To create a component, define a component called ``translations`` under
a new top-level ``components`` key. We will also add a new part that
dumps the contents of the ``translations`` directory:

.. literalinclude:: ../code/crafting/components/snapcraft.yaml
    :caption: snapcraft.yaml
    :language: yaml

Next, create a ``translations`` directory with a file called ``la``:

.. code-block:: text

   .
   ├── src
   │   └── my-app
   └── translations
       └── la

``la`` can also be an empty file.

Pack the snap with:

.. literalinclude:: ../code/crafting/components/task.yaml
    :language: bash
    :start-after: [docs:pack]
    :end-before: [docs:pack-end]
    :dedent: 2

This will produce 2 artifacts, the snap and the component:

* ``hello-components_1.0_amd64.snap``
* ``hello-components+translations_1.0.comp``

The ``my-app`` and ``la`` files are staged, primed, and packed in the snap
artifact. The component artifact has no payload. It is empty except for a
metadata file ``meta/component.yaml``.

To move the ``la`` translation file to the ``component`` artifact, use the
``organize`` key for the ``translations`` part:

.. literalinclude:: ../code/crafting/components-organize/snapcraft.yaml
    :caption: snapcraft.yaml
    :language: yaml

The parentheses around ``(component/translations)`` indicate that the files
should be organized into the translations component's install directory.

Pack the snap again with:

.. literalinclude:: ../code/crafting/components/task.yaml
    :language: bash
    :start-after: [docs:pack]
    :end-before: [docs:pack-end]
    :dedent: 2

This will produce two artifacts again but the component now contains the
``la`` translation file.

To upload the snap and the component to the store, specify the snap file
and the component file for the ``translations`` component.

.. code-block:: bash

   snapcraft upload hello-components_1.0_amd64.snap \
     --component translations=hello-components+translations_1.0.comp

The store expects a snap and its components to be uploaded together.

The component has to be uploaded alongside the snap.
