.. 8080.md

.. _supported-plugins:

Supported plugins
=================

The following plugins are currently supported by Snapcraft. See :ref:`Snapcraft plugins <snapcraft-plugins>` for more details in how they’re used, and to create your own, see :ref:`Writing local plugins <writing-local-plugins>`.

   ⓘ From Snapcraft 4.0 onwards, if the working directory contains a Snapcraft project, the default behaviour is to show only the plugins available for either its specified base or the latest available supported base (currently ``core22`` ). See :ref:`Base snaps <base-snaps>` for more details.

Programming languages
---------------------

Go
~~

.. list-table::
   :header-rows: 1

   * - Plugin name
     - Description
     - Base support
   * - :ref:`go <the-go-plugin>`
     - integrates projects written in Go and using the *go get* package installer
     - :ref:`core22 <the-go-plugin-core22>`
       :ref:`core20 <the-go-plugin-core20>`
       :ref:`core18 <the-go-plugin-core18>`
   * - :ref:`godeps <the-godeps-plugin>`
     - integrates projects written in Go and using the *godep* dependency tool
     - :ref:`core18 <the-godeps-plugin>`


Java
~~~~

.. list-table::
   :header-rows: 1

   * - Plugin name
     - Description
     - Base support
   * - :ref:`ant <the-ant-plugin>`
     - Ant build system integration, commonly used by Java projects
     - core22 core20 core18
   * - :ref:`gradle <the-gradle-plugin>`
     - integrate projects built using the Gradle build tool with your snaps
     - core18
   * - :ref:`maven <the-maven-plugin>`
     - build system integration with *Maven*, commonly used by Java projects
     - core22 core18


Node.js/JavaScript
~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Plugin name
     - Description
     - Base support
   * - :ref:`gulp <the-gulp-plugin>`
     - build parts from projects using the gulp.js streaming build system
     - core18
   * - :ref:`npm <the-npm-plugin>`
     - create parts that use Node.js and/or the JavaScript package manager, npm
     - :ref:`core22 <the-npm-plugin-core22>`
       :ref:`core20 <the-npm-plugin-core20>`
   * - :ref:`nodejs <the-nodejs-plugin>`
     - create parts that use Node.js and/or the JavaScript package manager, npm
     - :ref:`core18 <the-nodejs-plugin-core18>`


Python
~~~~~~

.. list-table::
   :header-rows: 1

   * - Plugin name
     - Description
     - Base support
   * - :ref:`conda <the-conda-plugin>`
     - used for parts incorporating the Conda open source package manager system
     - :ref:`core22 <the-conda-plugin-core22>`
       :ref:`core20 <the-conda-plugin-core20>`
       :ref:`core18 <the-conda-plugin-core18>`
   * - :ref:`python <the-python-plugin>`
     - used for parts incorporating projects written with Python 2 or Python 3
     - :ref:`core22 <the-python-plugin-core22>`
       :ref:`core20 <the-python-plugin-core20>`
       :ref:`core18 <the-python-plugin-core18>`


Other languages
~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Plugin name
     - Description
     - Base support
   * - :ref:`crystal <the-crystal-plugin>`
     - build parts from projects written in the Ruby-like Crystal language
     - :ref:`core20 <the-crystal-plugin-core20>`
       :ref:`core18 <the-crystal-plugin-core18>`
   * - :ref:`dotnet <the-dotnet-plugin>`
     - integrates with the Microsoft’s .NET SDK to build core runtime parts
     - :ref:`core22 <the-dotnet-plugin-core22>`
       :ref:`core18 <the-dotnet-plugin-core18>`
   * - :ref:`flutter <the-flutter-plugin>`
     - easily build and deploy parts for the expressive Flutter UI toolkit
     - :ref:`core22 <the-flutter-plugin-core22>`
       :ref:`core18 <the-flutter-plugin-core18>`
   * - :ref:`ruby <the-ruby-plugin>`
     - built parts from projects written in Ruby and its Gemfile dependency bundler
     - core18
   * - :ref:`rust <the-rust-plugin>`
     - build parts from projects written in Rust and using Cargo for dependency management
     - :ref:`core22 <the-rust-plugin-core22>`
       :ref:`core20 <the-rust-plugin-core20>`
       :ref:`core18 <the-rust-plugin-core18>`


Build tools
-----------

.. list-table::
   :header-rows: 1

   * - Plugin name
     - Description
     - Base support
   * - :ref:`autotools <the-autotools-plugin>`
     - integrates projects that use the common Autotools suite with your snaps
     - :ref:`core22 <the-autotools-plugin-core22>`
       :ref:`core20 <the-autotools-plugin-core20>`
       :ref:`core18 <the-autotools-plugin-core18>`
   * - :ref:`cmake <the-cmake-plugin>`
     - integrates projects that use the common CMake build tool with your snaps
     - :ref:`core22 <the-cmake-plugin-core22>`
       :ref:`core20 <the-cmake-plugin-core20>`
       :ref:`core18 <the-cmake-plugin-core18>`
   * - :ref:`make <the-make-plugin>`
     - integrates projects using the commonly found *make* build system
     - :ref:`core22 <the-make-plugin-core22>`
       :ref:`core20 <the-make-plugin-core20>`
       :ref:`core18 <the-make-plugin-core18>`
   * - :ref:`meson <the-meson-plugin>`
     - integrate projects build using the Meson build system into your snap
     - :ref:`core22 <the-meson-plugin-core22>`
       :ref:`core20 <the-meson-plugin-core20>`
       :ref:`core18 <the-meson-plugin-core18>`
   * - :ref:`qmake <the-qmake-plugin>`
     - integrates projects using the qmake build tool, commonly by *Qt*-based projects
     - :ref:`core20 <the-qmake-plugin-core20>`
       :ref:`core18 <the-qmake-plugin-core18>`
   * - :ref:`scons <the-scons-plugin>`
     - integrates projects that use the SCons construction tool
     - core22 core18
   * - :ref:`waf <the-waf-plugin>`
     - integrate projects using the Waf build automation tool
     - core18

Platforms
---------

Linux kernel
~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Plugin name
     - Description
     - Base support
   * - :ref:`kbuild <the-kbuild-plugin>`
     - build parts that use the Linux kernel build system (kBuild)
     - core18
   * - :ref:`kernel <the-kernel-plugin>`
     - derived from the *kbuild* plugin and used to build your own kernel
     - core18

Robot Operating System (ROS)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Plugin name
     - Description
     - Base support
   * - :ref:`ament <the-ament-plugin>`
     - uses ament_cmake to build parts for version 2 of the Robot Operating System (ROS 2)
     - core18
   * - :ref:`catkin <the-catkin-plugin>`
     - build catkin-based parts, typically used with version 1 of the Robot Operating System (ROS 1)
     - :ref:`core20 <the-catkin-plugin-core20>`
       :ref:`core18 <the-catkin-plugin-core18>`
   * - :ref:`catkin-tools <the-catkin-tools-plugin>`
     - alternative method for building projects using version 1 of the Robot Operating System (ROS 1)
     - :ref:`core20 <the-catkin-tools-plugin-core20>`
       :ref:`core18 <the-catkin-tools-plugin-core18>`
   * - :ref:`colcon <the-colcon-plugin>`
     - build colcon-based parts, typically used with version 2 of the Robot Operating System (ROS 2)
     - :ref:`core22 <the-colcon-plugin-core22>`
       :ref:`core20 <the-colcon-plugin-core20>`
       :ref:`core18 <the-colcon-plugin-core18>`

Tools
-----

.. list-table::
   :header-rows: 1

   * - Plugin name
     - Description
     - Base support
   * - :ref:`dump <the-dump-plugin>`
     - simply dumps the contents from the specified source
     - core22 core20 core18
   * - :ref:`nil <the-nil-plugin>`
     - useful for parts with no source to import
     - core22 core20 core18
   * - :ref:`plainbox-provider <the-plainbox-provider-plugin>`
     - create parts containing a Plainbox test collection known as a *provider*
     - core18

.. toctree::
   :hidden:

   reference-core22
   reference-core20
   reference-core18
