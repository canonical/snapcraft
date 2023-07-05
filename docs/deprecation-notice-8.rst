.. 8408.md

.. _deprecation-notice-8:

Deprecation notice: 8
=====================

**The build keyword has been replaced by override-build**

*introduced in snapcraft 2.41*

The ``build`` scriptlet was originally introduced as a way to override the default ``build`` step with one’s own logic.

However, as part of an effort to add support for Snapcraft to have this for *all* lifecycle steps (i.e. not just build), a new scriptlet has been added that works similarly called ``override-build``.

The biggest difference is that you can now call ``snapcraftctl build`` to run the default ``build`` step.

See :ref:`Deprecation notices <deprecation-notices>` for further announcements.
