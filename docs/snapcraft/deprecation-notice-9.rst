.. 8409.md

.. _deprecation-notice-9:

Deprecation notice: 9
=====================

**The install keyword has been replaced by override-build**

*introduced in snapcraft 2.41*

The ``install`` scriptlet was originally introduced as a way to “install” artefacts after a build.

However, as part of an effort to add support for Snapcraft to have this for *all* lifecycle steps (i.e. not just build), a new scriptlet has been added that encompasses this functionality called ``override-build``.

``override-build`` allows you to override the default ``build`` step with your own logic, from which you can call ``snapcraftctl build`` to run the default ``build`` step.

Let’s say you currently had an ``install`` scriptlet that looked like this:

.. code:: yaml

   install: |
     echo "This runs after build!"

To get equivalent functionality with the ``override-build`` scriptlet, try this:

.. code:: yaml

   override-build: |
     snapcraftctl build
     echo "This runs after build!"

See :ref:`Deprecation notices <deprecation-notices>` for further announcements.
