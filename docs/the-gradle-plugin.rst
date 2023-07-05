.. 5390.md

.. _the-gradle-plugin:

The Gradle plugin
=================

This plugin is used for building parts that use gradle.

The gradle build system is a popular build tool used to build Java projects. The plugin requires that gradle’s build.gradle file exists in the root of the source tree.

   ⓘ This plugin is only available to *core* and *core18* based snaps. See :ref:`Base snaps <base-snaps>` for details.

See the :ref:`Java Applications <java-applications>` tutorial for a complete and annotated example.

This plugin uses the common plugin keywords as well as those for “sources”. For more information, see :ref:`Snapcraft parts metadata <snapcraft-parts-metadata>`.

Additionally, this plugin uses the following plugin-specific keywords

-  ``gradle-options`` (list of strings) flags to pass to the build using the gradle semantics for parameters. The ‘jar’ option is automatically passed and will be passed in as the last parameter. Essentially you can include any gradle command line options such as ‘-x test’ ‘–debug’ as a comma separated list.

   .. code:: yaml

        gradle-options: [-xtest, --debug]

   Note: your options must not contain spaces!

-  ``gradle-output-dir`` (string; default: ‘build/libs’) Informs snapcraft where your build.gradle will place the generated files. The output directory where the resulting jar or war files from gradle[w] are generated. You should normally NOT include this option unless your build.gradle is placing the output files in an unusual place.

   .. code:: yaml

          gradle-output-dir: build/libs

-  ``gradle-version`` (string) The version of gradle you want to use to build the source artifacts. Defaults to the current release downloadable from https://services.gradle.org/distributions/ The entry is ignored if gradlew is found.

-  ``gradle-version-checksum`` (string) The checksum for gradle-version in the form of /. As an example “sha512/2a803f578f341e164f6753e410413d16ab60fab…”.

-  ``gradle-openjdk-version`` (string) openjdk version available to the base to use. If not set the latest version available to the base will be used.

Building Wars
-------------

The gradle plugin currently always runs the ``jar`` task rather than the ``build`` task. The result is that even if your build.gradle is configured to build a war you will actually end up with a jar. To fix this problem include a gradle-option keyword with the ‘war’ task.

.. code:: yaml

       gradle-options: [-xtest, war]

The jar file will still be generated but so will your war file.

gradlew vs gradle
-----------------

When the gradle plugin runs it will search for ‘gradlew’ in the directory you run snapcraft from. If it finds gradlew then it will run the build using gradlew. If gradlew doesn’t exist then it will fall back to using gradle. Using gradlew is recommended. If you are looking to run the latest version of gradle then make certain that gradlew is present.

Proxy settings
--------------

The gradle plugin will automatically go looking for environment variables that contain proxy settings. The environment variable names that it looks for are:

::

   https_proxy.proxyHost
   https_proxy.proxyPort
   https_proxy.proxyUser
   https_proxy.proxyPassword

   http_proxy.proxyHost
   http_proxy.proxyPort
   http_proxy.proxyUser
   http_proxy.proxyPassword

If found these variables will be passed as options to gradle.

Examples
--------

.. code:: yaml

   parts:
     irrigation-webapp:
       plugin: gradle
       source: https://github.com/bsutton/IrrigationForPi.git
       source-type: git
       gradle-options: [-xtest, war]   # suppress running of tests and run the war task
       gradle-output-dir: build/libs

`Examples on GitHub <https://github.com/search?o=desc&q=path%3Asnapcraft.yaml+%22plugin%3A+gradle%22+&s=indexed&type=Code&utf8=%E2%9C%93>`__

   ⓘ This is a *snapcraft* plugin. See :ref:`Snapcraft plugins <snapcraft-plugins>` and :ref:`Supported plugins <supported-plugins>` for further details on how plugins are used.
