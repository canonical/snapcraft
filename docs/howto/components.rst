How to package and upload a snap with components
************************************************

.. include:: /reuse/components-intro.rst

Start with a simple ``snapcraft.yaml``:

.. literalinclude:: code/basic/snapcraft.yaml
    :language: yaml

And create the following project tree:

.. code-block::

   .
   └── src
       └── my-app

``my-app`` can be an empty file. The contents are not important for this
how-to guide.

To create a component, define a component called ``translations`` under
a new top-level ``components`` keyword. We will also add a new part that
dumps the contents of the ``translations`` directory:

.. literalinclude:: code/components/snapcraft.yaml
    :language: yaml

Next, create a ``translations`` directory with a file called ``la``:

.. code-block::

   .
   ├── src
   │   └── my-app
   └── translations
       └── la

``la`` can also be an empty file.

Pack the snap with:

.. literalinclude:: code/components/task.yaml
    :language: shell
    :start-after: [docs:pack]
    :end-before: [docs:pack-end]
    :dedent: 2

This will produce 2 artefacts, the snap and the component:

* ``hello-components_1.0_amd64.snap``
* ``hello-components+translations_1.0.comp``

The ``my-app`` and ``la`` files are staged, primed, and packed in the snap
artefact. The component artefact has no payload. It is empty except for a
metadata file ``meta/component.yaml``.

To move the ``la`` translation file to the ``component`` artefact, use the
``organize`` keyword for the ``translations`` part:

.. literalinclude:: code/components-organize/snapcraft.yaml
    :language: yaml

The parentheses around ``(component/translations)`` indicate that the files
should be organized into the translations component's install directory.

Pack the snap again with:

.. literalinclude:: code/components/task.yaml
    :language: shell
    :start-after: [docs:pack]
    :end-before: [docs:pack-end]
    :dedent: 2

This will produce two artefacts again but the component now contains the
``la`` translation file.

To upload the snap and the component to the store, specify the snap file
and the component file for the ``translations`` component.

.. code-block:: shell

   snapcraft upload hello-components_1.0_amd64.snap \
     --component translations=hello-components+translations_1.0.comp

The store expects a snap and its components to be uploaded together.

The component has to be uploaded alongside the snap.
