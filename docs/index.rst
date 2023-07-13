.. 30905.md

.. _snapcraft-documentation:

Snapcraft documentation
=======================

.. note::
          **NOTE TO EDITORS**

          This topic is currently under construction as we start to migrate our current documentation to use the `Diátaxis <https://diataxis.fr/>`__ framework.

          See `Diátaxis, a new foundation for Canonical documentation <https://ubuntu.com/blog/diataxis-a-new-foundation-for-canonical-documentation>`__ for more details.



Snapcraft **builds**, **packages** and **publishes** snaps. It’s available for **Ubuntu**, many **other Linux distributions**, and **macOS**, and runs from the command line.

With Snapcraft, developers can use platform-specific **plugins** and **extensions** to **streamline** and **simplify** the build process. Snaps can then be **tested and shared locally** before being published to the global `Snap Store <https://snapcraft.io/store>`__ within channels, tracks and branches to **finely control releases**.

Snapcraft does all of this within a self-launched `LXD <https://linuxcontainers.org/lxd/docs/master/>`__ or `Multipass <https://multipass.run/docs>`__ **container**, or **natively** from your own environment, and includes **linting, debug and staging functions** to help solve problems. You can also incorporate Snapcraft package building into your CI systems, and **build snaps remotely** from either **GitHub** or our **Launchpad** build farm.

From desktop applications, servers and cloud deployments, to embedded devices and IoT. From a single user to hundreds of thousands of users, Snapcraft can help.

--------------

In this documentation
---------------------

+-----------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------+
| :ref:`Tutorials <snapcraft-tutorials>`\  Get started - a hands-on introduction to building snaps with Snapcraft | :ref:`How-to guides <snapcraft-how-to-guides>` Step-by-step guides covering key operations and common tasks |
+-----------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------+
| :ref:`Reference <snapcraft-reference>` Technical information - plugins, extensions and architecture             | :ref:`Explanation <snapcraft-explanation-guides>` Concepts - discussion and clarification of key topics     |
+-----------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------------------------------------------------------+

--------------

Project and community
---------------------

Snap and Snapcraft are members of the Ubuntu family. They’re both open source projects that welcome community involvement, contributions, suggestions, fixes and constructive feedback.

-  `Our Code of Conduct <https://ubuntu.com/community/code-of-conduct>`__
-  `Get support <https://forum.snapcraft.io/c/snap/14>`__
-  `Join the Discourse forum <https://forum.snapcraft.io/>`__
-  :ref:`How to contribute <documentation-guidelines>`
-  `Roadmap <https://snapcraft.io/docs/the-snapd-roadmap>`__

Thinking about using snap for your next project? `Get in touch! <https://forum.snapcraft.io/>`__


License
-------

.. image:: images/cc-by-nc-sa-3.0-88x31.png
   :alt: License: CC BY-NC-SA 3.0

This documentation is licensed under a `Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License`_.


.. toctree::
   :hidden:

   snapcraft-tutorials
   snapcraft-how-to-guides
   snapcraft-reference
   snapcraft-explanation-guides
