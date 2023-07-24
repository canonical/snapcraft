.. 4284.md

.. _snapcraft-plugins:

Plugins
=======

Plugins are used by the *snapcraft* command to build a snap from *parts* defined within ``snapcraft.yaml``.

Commonly used plugins include *Python*, *Go*, *Java*, *cmake* and *autotools*, and these help when working with projects written in a specific language or with a specific set of build tools.

These, and many other plugins, are included with Snapcraft, all of which can be listed with the following command:

.. code:: bash

   $ snapcraft list-plugins
   Displaying plugins available for 'core20'
   autotools  cmake  dump  go  make  meson  nil  npm  python  rust

With Snapcraft 4.0, if the working directory contains a Snapcraft project, the default behaviour is to show only the plugins available for either its specified base or the latest available supported base (currently ``core20``).

To list plugins specific to a defined base, run the following command:

.. code:: bash

   $ snapcraft list-plugins --base core18
   Displaying plugins available for 'core18'
   ant           cmake    dotnet   godeps  make   nodejs             ruby
   autotools     colcon   dump     gradle  maven  plainbox-provider  rust
   catkin        conda    flutter  kbuild  meson  python             scons
   catkin-tools  crystal  go       kernel  nil    qmake              waf

Further information about any specific plugin can be obtained by typing ``snapcraft help`` followed by the plugin name:

.. code:: bash

   $ snapcraft help python

With Snapcraft 4.0, the help command is also base aware. To get help for a plugin targeting a specific base, run:

.. code:: bash

   $ snapcraft help python --base core18

For further details on specific plugins, see :ref:`Supported plugins <supported-plugins>`, and to create your own, see :ref:`Writing local plugins <writing-local-plugins>`.

