.. 8397.md

.. _deprecation-notice-1:

Deprecation notice: 1
=====================

**The snap keyword has been replaced by prime.**

Snapcraft supports providing a list of files to include/exclude from two steps in the lifecycle: stage and prime. That’s done with two keywords in the yaml, respectively: ``stage`` and ``snap``. The ``snap`` keyword has now been deprecated and replaced with ``prime``, corresponding to the actual lifecycle step to which it applies. All other functionality remains the same. Where you wrote:

.. code:: yaml

   # ...
   parts:
     foo:
       plugin: nil
       snap: [bar]

Now write:

.. code:: yaml

   # ...
   parts:
     foo:
       plugin: nil
       prime: [bar]

See :ref:`Deprecation notices <deprecation-notices>` for further announcements.
