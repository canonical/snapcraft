.. 33074.md

.. _basic-snapcraft-yaml-example:

Basic snapcraft.yaml example
============================

The following is a complete example :file:`snapcraft.yaml` file taken from our :ref:`Python documentation <python-apps>`. It’s for an application called `yt-dlp <https://github.com/yt-dlp/yt-dlp>`__, a command line tool for parsing online videos.

We will look at each section of the code separately and learn what it does:

::

   name: yt-dlp
   summary: A fork of youtube-dl with additional features and patches
   description: |
         Download and play videos on your local system. Runs from the command
         line and with all the features and patches of youtube-dlc in addition
         to the latest youtube-dl.
   version: test
   grade: stable
   confinement: strict
   base: core22

   apps:
     yt-dlp:
       command: bin/yt-dlp
       plugs: [home, network, network-bind, removable-media]

   parts:
     yt-dlp:
       plugin: python
       source: https://github.com/yt-dlp/yt-dlp.git


.. _basic-snapcraft-yaml-example-metadata:

Metadata
--------

Several :ref:`mandatory fields <snapcraft-top-level-metadata>` define and describe the application - name, version, summary and description. These fields allow end users to find snaps and install them.

-  **Name** - A string that defines the name of the snap. It must start with an ASCII character and can only use ASCII lowercase letters, numbers and hyphens. It must be between 1 and 40 characters in length. The name must be unique in the Snap Store.
-  **Version** - A string that defines the version of the application to the user. Max. length is 32 characters.
-  **Summary** - A string that briefly describes the application. The summary can not exceed 79 characters. You can use a chevron ‘>’ character in this key to declare a multi-line summary.
-  **Description** - A string that describes the application. You can use multiple lines. There is no practical limitation on the length of this key.

=============== =========== ====== ====== ===================
Field           Definition  Type   Length Note
=============== =========== ====== ====== ===================
**Name**        Name        String 1-40   Unique
**Version**     App version String 1-32   No semantic meaning
**Summary**     Brief       String 1-79   -
**Description** Full        String High   -
=============== =========== ====== ====== ===================

When the application is built with snapcraft, this metadata will be made available to users. It can also be used to pre-populate certain fields in the snap’s Snap Store page during upload.


.. _basic-snapcraft-yaml-example-base:

Base
----

As part of their security design, by default, snaps cannot see the root filesystem on the host. This prevents conflict with other applications and increases security. However, applications still need some location to act as their root filesystem. Furthermore, they would also benefit from common libraries (e.g. libc) being in this root filesystem rather than being bundled into each application.

The :ref:`base keyword <base-snaps>` specifies a special kind of snap that provides a minimal set of libraries common to most applications. It is mounted and used as the root filesystem for the applications inside the snap. In essence, this means the snaps behave as though they were running on a system that matches the base declaration.

Several bases are available, including core18, core20, core22, etc. These bases match the Ubuntu LTS releases, e.g.: core20 library set is equivalent to Ubuntu 20.04. For most practical purposes, the use of either ``core20`` or ``core22`` is recommended, depending on the :ref:`Supported plugins <supported-plugins>` you wish to use.

.. code:: yaml

   base: core22


.. _basic-snapcraft-yaml-example-confinement:

Confinement
-----------

Security confinement distinguishes snaps from software distributed using the traditional repository methods. The :ref:`confinement mechanism <snap-confinement>` allows for a high level of isolation and security, and prevents snaps from being affected by underlying system changes, one snap another, or snaps affecting the host system.

Different confinement levels describe what type of access the snap applications will have once installed on the user’s system. Confinement levels can be treated as filters that define what type of system resources the application can access outside the snap.

Confinement is defined by general levels and fine-tuned using interfaces.

There are three levels of confinement:

-  Strict - This confinement level uses Linux kernel security features to lock down the applications inside the snap. By default, a strictly confined application cannot access the network, the users’ home directory, any audio subsystems or webcams, and it cannot display any graphical output via X or Wayland.
-  Devmode - This is a debug mode level used by developers as they iterate on the creation of their snap. This allows developers to troubleshoot applications, because they may behave differently when confined.
-  Classic - This is a permissive level equivalent to the full system access that traditionally packaged applications have. Classic confinement is often used as a stop-gap measure to enable developers to publish applications that need more access than the current set of permissions allow. The classic level should be used only when required for functionality, as it lowers the security of the application. Classically confined snaps are reviewed by the Snap Store reviewers team before they can be published. Snaps that use classic confinement may be rejected if they don’t meet the necessary requirements.

================== =================== =============== ===============
Access type        Strict              Devmode         Classic
================== =================== =============== ===============
Access to network  N                   Y               System
Access to home dir N                   Y               System
Access to audio    N                   Y               System
Access to webcam   N                   Y               System
Access to display  N                   Y               System
Used for           Preferred           Troubleshooting Stopgap measure
Other              Interfaces override -               Requires review
================== =================== =============== ===============

The xsv snap has its confinement level set as strict:

.. code:: yaml

   confinement: strict


.. _basic-snapcraft-yaml-example-interfaces:

Interfaces
----------

A strictly confined snap is considered untrusted, and it runs in a restricted sandbox. By design, untrusted applications:

-  can freely access their own data.
-  cannot access other applications data.
-  cannot access non-application-specific user data.
-  cannot access privileged portions of the OS.
-  cannot access privileged system APIs.
-  may access sensitive APIs under some conditions.

Strictly confined applications are not always functional with the default security policy. For example, a browser without network access or a media player without audio access do not serve their intended purpose.

To that end, snap developers can use `interfaces <https://snapcraft.io/docs/snapcraft-interfaces>`__, a mechanism of granular resource-level security permissions. These allow developers to expand on the default security policies and connect their applications to system resources. The declarations are provided at build time in the snapcraft.yaml file.

An interface consists of a connection between a slot and a plug. The slot is the provider of the interface while the plug is the consumer, and a slot can support multiple plug connections.

.. figure:: https://assets.ubuntu.com/v1/59c290a8-snapd-interfaces.png
   :alt: How an interface uses a plug and a slot


Interfaces can be automatically or manually connected. Some interfaces will be auto-connected. Others may not, especially if they have access to sensitive resources (like network control, for instance). Users have the option to manually control interfaces – connect and disconnect them.


.. _basic-snapcraft-yaml-example-build:

Build definition
~~~~~~~~~~~~~~~~

The build definition stanza comprises the apps and parts section of the snapcraft.yaml. These two sections describe how the application is going to be built, what sources and options will be used, and what permissions it will have to run (in relation to the snap’s security confinement).

-  The :ref:`parts <adding-parts>` section defines all the sources that will be used to build the applications inside the snaps.
-  The :ref:`apps <snapcraft-app-and-service-metadata>` section defines the command path for each application (how it will be run), optional parameters, as well as the list of permissions (plugs to interfaces) that will be granted to the application at runtime.

.. _basic-snapcraft-yaml-example-parts:

The parts definition
--------------------

The parts definition consists of the following lines of code:

.. code:: yaml

   parts:
     yt-dlp:
       plugin: python
       source: https://github.com/yt-dlp/yt-dlp.git

The yt-dlp snap only has one part. It is built using the Python plugin, which is a Snapcraft plugin designed to simplify the building of Python applications.

-  plugin: This block defines the use of the Snapcraft Python plugin that will perform various language-specific commands in the background. The :ref:`python plugin <the-python-plugin>` handles Python building and its dependencies automatically. The plugin declaration has only one sub-section:

   -  source: defines the URL or a path of the application code that needs to be downloaded for the build. It can be a local or remote path, and can refer to a directory tree, a compressed archive or a revision control repository. In this particular case, the application is built the project’s upstream GitHub repository.

.. _basic-snapcraft-yaml-example-apps:

The apps definition
-------------------

The apps build definition consists of the following lines of code:

.. code:: yaml

   apps:
     yt-dlp:
       command: bin/yt-dlp
       plugs: [home, network, network-bind, removable-media]

The yt-dlp example has a single application - yt-dlp. Other snaps may have multiple sub-applications or executables.

-  command: defines the path to the executable (relative to the snap) and arguments to use when this application runs.
-  plugs: defines the list of interfaces to which the app will have access to. This enables the intended application functionality. In this specific case, the yt-dlp snap will be allowed access to the home, network and removable-media interfaces, which are not available by default under strict confinement. This will allow the user of the tool to access files in the user’s home directory, from a network connection, or from any mounted removable media locations.

The next step in the process is to build the snap. However, before we do that, let’s examine a more complex snap.
