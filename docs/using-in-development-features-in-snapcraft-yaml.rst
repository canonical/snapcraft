.. 5766.md

.. _using-in-development-features-in-snapcraft-yaml:

Using in-development features in ``snapcraft.yaml``
===================================================

When in need to be able to use new features of ``snapd`` as soon as they are available, and in fact even before they are available (edge, beta, etc), but has not made it into ``snapcraft``\ ’s validation mechanisms yet there is an option of using the ``passthrough`` attribute in ``snapcraft.yaml`` which has the following properties:

-  snapcraft will accept two new fields, named ``passthrough``. One at the top level, and one at the application scope. In points below, these locations are referred to as the target scope.
-  When these fields exist they must necessarily contain a map.
-  Options inside the map are passed through directly into “snap.yaml”, in the target scope where the passthrough option itself lives.
-  snapcraft must forbid defining an option inside the passthrough if that option is also directly present in the target scope.
-  snapcraft must warn whenever it sees an option in the passthrough, to remind the publisher that these options are not yet supported.
-  snapcraft might learn to report options that are potentially already supported. But it must not prevent these options from being passed through, since the option may be defined in a way that is not yet understood by snapcraft and should thus pass through.

The format is as follows in way of an example:

::

   name: foo
   passthrough:
       confinement: next-generation
       planet: saturn
   apps:
       foo:
           command: foo
           passthrough:
               daemon: complex

This would allow use of the *fictional*, not yet ready, - ``next-generation`` value for ``confinement``. - a new ``planet`` attribute with a value of ``saturn``. - a ``complex`` value for ``daemon`` for the app ``foo``.
