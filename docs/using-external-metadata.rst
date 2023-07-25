.. 4642.md

.. _using-external-metadata:

Using external metadata
=======================

To help avoid unnecessary duplication, and for convenience, Snapcraft can process and incorporate external metadata from within :file:`snapcraft.yaml` by using ``parse-info`` within a :ref:`part <adding-parts>` and a corresponding ``adopt-info`` key.

For example, the following ``snapcraft.yaml`` will parse a file called ``metadata-file``. Snapcraft will attempt to extract ``version``, ``summary`` and ``description`` metadata for the snap, all of which are mandatory:

.. code:: yaml

   name: my-snap-name
   adopt-info: part-with-metadata

   parts:
     part-with-metadata:
       plugin: dump
       source: .
       parse-info: [metadata-file]

See :ref:`The snapcraft format <the-snapcraft-yaml-schema>` for further details on Snapcraft metadata and how it’s used.

Source types
------------

An external metadata source can be one of the following:

-  `AppStream <meta-appstream_>`__: a standard for software components
-  `Scriptlets <meta-scriptlet_>`__: a *snapcraftctl*-driven command to generate ``version`` and ``grade``

See below for details on incorporating each of the above into your :file:`snapcraft.yaml` file.


.. _using-external-metadata-meta-appstream:
.. _meta-appstream:

AppStream
~~~~~~~~~

`AppStream`_ is a metadata standard used to describe a common set software components. It can be parsed by *snapcraft* to provide the ``title``, ``version``, ``summary``, ``description`` and ``icon`` for a snap, along with the location of an app’s :ref:`desktop <desktop-files-for-menu-integration>` file.

The following is a typical example from an upstream project. It’s an *AppStream* file called ``sampleapp.metainfo.xml``:

.. code:: xml

   <?xml version="1.0" encoding="UTF-8"?>
   <component type="desktop-application">
     <id>com.example.sampleapp</id>
     <name>Sample App</name>
     <summary>Single-line elevator pitch for your amazing application</summary>
     <description>
       This is applications's description. A paragraph or two to tell the
       most important story about it.
     </description>
     <icon type="local">assets/icon.png</icon>
     <launchable type="desktop-id">
       com.example.sampleapp.desktop
     </launchable>
     <releases>
       <release date="2019-11-27" version="4.2.8.0"/>
     </releases>
   </component>

We *adopt* the above metadata into ``snapcraft.yaml`` with the following:

.. code:: yaml

   name: sampleapp-name
   adopt-info: sampleapp

   apps:
     sampleapp:
       command: sampleapp
       common-id: com.example.sampleapp

   parts:
     sampleapp:
       plugin: dump
       source: http://github.com/example/sampleapp.git
       parse-info: [usr/share/metainfo/com.example.sampleapp.appdata.xml]

..

   ⓘ The path in ``parse-info`` is a relative path from the part source, build or install directory (:ref:`SNAPCRAFT_PART_SRC, SNAPCRAFT_PART_BUILD, SNAPCRAFT_PART_INSTALL <parts-lifecycle-parts-directories>`).

The resulting snap will use the title, version, summary and description from the AppStream file.

You can also link each app in your snap to specific AppStream metadata by pointing the ``common-id`` key of that app to the *component id* field in the AppStream metadata. Snapcraft will use the metadata of that component to get the ``.desktop`` entry file for that app.

   ⓘ For backwards compatibility, some component ids in the AppStream metadata have a ``.desktop`` suffix. If this is the case for your application, the ``common-id`` of your app should also use that suffix.

**Note:** The process to get the ``.desktop`` file entry from the AppStream metadata goes as follows. First, Snapcraft searches for a parsed AppStream file with the same*\ component id\* as the app’s ``common-id`` and extracts the `Desktop File ID <https://specifications.freedesktop.org/desktop-entry-spec/desktop-entry-spec-latest.html#desktop-file-id>`__ (``desktop-id``) from that component. If that component doesn’t specify a ``desktop-id``, Snapcraft will use the *component id* as the Desktop File ID. Snapcraft will then search for a desktop file matching the Desktop File ID in the ``usr/local/share`` and ``usr/share`` directories relative to the part source, and by following the `Desktop File ID <https://standards.freedesktop.org/desktop-entry-spec/desktop-entry-spec-latest.html#desktop-file-id>`__ rules.


.. _using-external-metadata-meta-scriptlet:
.. _meta-scriptlet:

Part scriptlets
~~~~~~~~~~~~~~~

Individual parts in your ``snapcraft.yaml`` can set the ``version`` and ``grade`` by using ``snapcraftctl``. All you need to do is select which part to adopt using ``adopt-info``:

.. code:: yaml

   # ...
   adopt-info: my-part
   # ...
   parts:
     my-part:
       # ...
       override-pull: |
         snapcraftctl pull
         snapcraftctl set-version "my-version"
         snapcraftctl set-grade "devel"

See :ref:`Scriptlets <override-build-steps>` for more details on using scripting elements within :file:`snapcraft.yaml`.


.. _using-external-metadata-setup-py:

[not recommended] ``setup.py``\
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

..

   ⚠ Using ``parse-info`` with ``setup.py`` is currently discouraged because it has many issues. For example, it incorrectly `uses the project’s summary as the snap’s description <https://bugs.launchpad.net/snapcraft/+bug/1813364>`__ and it `might crash the snap build <https://github.com/snapcore/snapcraft/pull/2756#issuecomment-544284814>`__.

A `setup.py <https://docs.python.org/3/distutils/setupscript.html>`__ file is used by many Python projects to help with package installation. If your *setup.py* uses `setuptools`_ and defines ``version`` and ``description``, these can be extracted from ``setup.py`` and used as the ``version`` and ``description`` metadata in the resulting snap.

The following is an example ``setup.py`` in the root of a hypothetical git tree:

.. code:: python

   import setuptools

   setuptools.setup(
       name='hello-world',
       version='1.0',
       author='snapcrafter',
       author_email='snapcraft@lists.snapcraft.io',
       description='A simple hello world in python',
       scripts=['hello']
   )

You can *adopt* the relevant metadata in the above with the following snapcraft.yaml

.. code:: yaml

   name: sampleapp-name
   summary: sampleapp summary
   adopt-info: sampleapp

   apps:
     sampleapp:
       command: sampleapp

   parts:
     sampleapp:
       plugin: python
       source: http://github.com/example/sampleapp.git
       parse-info: [setup.py]


.. _using-external-metadata-version:

Snapcraft versions and compatibility
------------------------------------

======================== =================
Change                   snapcraft version
======================== =================
Initial introduction     ``2.39``
appstream support        ``2.39``
``common-id``            ``2.40``
``setup.py`` support     ``2.41``
snapcraftctl set-version ``2.41``
snapcraftctl set-grade   ``2.41``
======================== =================

.. _appstream: https://www.freedesktop.org/software/appstream/docs/
