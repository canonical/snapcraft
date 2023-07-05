.. 8407.md

.. _deprecation-notice-7:

Deprecation notice: 7
=====================

**The prepare keyword has been replaced by override-build**

*introduced in snapcraft 2.41*

The ``prepare`` scriptlet was originally introduced as a way to “prepare” for a build. However, as part of an effort to add support for Snapcraft to have this for *all* lifecycle steps (i.e. not just build), a new scriptlet has been added that encompasses this functionality called ``override-build``.

   ⓘ It’s also possible to replace ``prepare`` with :ref:`override-pull <override-build-steps>`, depending on your requirements.

``override-build`` allows you to override the default ``build`` step with your own logic, from which you can call ``snapcraftctl build`` to run the default ``build`` step.

Let’s say you currently had a ``prepare`` scriptlet that looked like this:

::

   prepare: |
     echo "This runs before build!"

To get equivalent functionality with the ``override-build`` scriptlet, try this:

::

   override-build: |
     echo "This runs before build!"
     snapcraftctl build

See :ref:`Deprecation notices <deprecation-notices>` for further announcements.
