.. 8628.md

.. _the-qmake-plugin:

The qmake plugin
================

The ``qmake`` plugin is useful when building `qmake <http://doc.qt.io/qt-5/qmake-manual.html>`__-based parts.

Qmake-based projects typically use `Qt <https://www.qt.io/>`__, although not always, and are built using *.pro* files.

This plugin uses the common plugin keywords as well as those for :ref:`sources <snapcraft-parts-metadata-source>`. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

Additional features and syntax are dependent on which :ref:`base <base-snaps>` is being used, as outlined below:

-  `base: core20 <the-qmake-plugin-core20_>`__
-  `base: core18 \| core <the-qmake-plugin-core18_>`__

For examples, search `GitHub <https://github.com/search?q=path%3Asnapcraft.yaml+%22plugin%3A+qmake%22&type=Code>`__ for projects already using the plugin.

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.


.. _the-qmake-plugin-core20:

base: core20
~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **qmake-parameters** (list of strings) additional options to pass to the qmake invocation.
-  **qmake-project-file** (string) the qmake project file to use. This is usually only needed if qmake can not determine what project file to use on its own.


.. _the-qmake-plugin-core18:

base: core18 \| core
~~~~~~~~~~~~~~~~~~~~

This plugin uses the following plugin-specific keywords:

-  **options** (list of strings) additional options to pass to the qmake invocation.
-  **qt-version** (string; default: qt5) Version of Qt to use with qmake. Valid options are: ``qt4`` and ``qt5``.
-  **project-files** (list of strings) list of .pro files to pass to the qmake invocation.
