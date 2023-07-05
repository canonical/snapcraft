.. 5124.md

.. _snapcraft-plugin-api:

Snapcraft plugin API
====================

The process of creating a snap follows a specific :ref:`lifecycle <parts-lifecycle>` for each part. What happens during each step in the lifecycle depends on the plugin being used for that part.

``BasePlugin``
--------------

Each plugin in Snapcraft inherits from the ``snapcraft.BasePlugin``.

Attributes
~~~~~~~~~~

``BasePlugin.build_snaps``
^^^^^^^^^^^^^^^^^^^^^^^^^^

Set of ``build-snaps`` required for this part.

``BasePlugin.build_packages``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Set of ``build-packages`` required for this part.

``BasePlugin.stage_packages``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Set of ``stage-packages`` required for this part.

``BasePlugin.project``
^^^^^^^^^^^^^^^^^^^^^^

``snapcraft.Project`` instance containing details about the project being built.

``BasePlugin.options``
^^^^^^^^^^^^^^^^^^^^^^

A dynamically-generated class containing the values of the schema-defined properties (note that hyphens in the schema are converted to underscores, e.g. ``stage-packages`` becomes ``stage_packages``).

``BasePlugin.partdir``
^^^^^^^^^^^^^^^^^^^^^^

The working directory specific to this part.

``BasePlugin.sourcedir``
^^^^^^^^^^^^^^^^^^^^^^^^

The directory to which sources are pulled.

``BasePlugin.builddir``
^^^^^^^^^^^^^^^^^^^^^^^

The directory in which the part is built.

``BasePlugin.installdir``
^^^^^^^^^^^^^^^^^^^^^^^^^

The directory into which build artifacts for this part are installed.

``BasePlugin.PLUGIN_STAGE_SOURCES``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Alternative collection of Debian repositories to use for ``stage-packages`` (by default, the host’s configured repositories is used). This is one long string with line breaks between repositories, for example:

.. code:: python

   self.PLUGIN_STAGE_SOURCES = """\
   deb http://us.archive.ubuntu.com/ubuntu/ xenial main restricted
   deb http://us.archive.ubuntu.com/ubuntu/ xenial-updates main restricted
   deb http://security.ubuntu.com/ubuntu xenial-security main restricted
   """

Methods
~~~~~~~

``BasePlugin.__init__(name, options, project=None)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Create a new Snapcraft plugin instance for a part with a specific ``name`` and set of ``options`` (which is a ``class``).

``BasePlugin.pull()``
^^^^^^^^^^^^^^^^^^^^^

Pull the source code and/or internal prerequisites to build the part.

``BasePlugin.clean_pull()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Clean the pulled source for this part.

``BasePlugin.build()``
^^^^^^^^^^^^^^^^^^^^^^

Build the source code retrieved during the ``pull`` step.

``BasePlugin.clean_build()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Clean the artifacts that resulted from building this part.

``BasePlugin.env(root)``
^^^^^^^^^^^^^^^^^^^^^^^^

Return a list with the execution environment for building and running from the ``root`` directory. Each item in the list should be a string of the form “VARIABLE=VALUE”.

``@classmethod``\ \ ``BasePlugin.schema(cls)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Return a JSON schema for the plugin’s properties as a dictionary. Of particular interest to plugin authors is probably the ``properties`` and ``required`` keywords, which outline the properties for this plugin as well as which ones are required.

``@classmethod``\ \ ``BasePlugin.get_pull_properties(cls)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Return a list of schema properties that are used by the ``pull`` step which, if changed, will require the ``pull`` step to be re-run.

``@classmethod``\ \ ``BasePlugin.get_build_properties(cls)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Return a list of schema properties that are used by the ``build`` step which, if changed, will require the ``build`` step to be re-run.
