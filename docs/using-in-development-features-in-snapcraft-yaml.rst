.. 5766.md

.. _using-in-development-features-in-snapcraft-yaml:

Using in-development features in snapcraft.yaml
===============================================

If you need to use a new feature of ``snapd`` as soon as it is available, but
before it is supported by the validation mechanisms in Snapcraft, you have the
option of using the passthrough feature of ``snapcraft.yaml`` to specify the
in-development feature you need.

The passthrough keyword
-----------------------

Snapcraft can accept the ``passthrough`` keyword in two places in the
``snapcraft.yaml`` file. It can be used at the top-level scope to access new
top-level features, and also at the application scope to access new
application-level features. Where either or these locations are used, they are
referred to as the *target scope*.

The ``passthrough`` keyword defines a map whose keywords and values are copied
directly into the corresponding scope in the ``snap.yaml`` description that
``snapd`` uses.

Although Snapcraft does not know about in-development features, it performs
some checks to prevent accidental conflicts or unexpected behaviour as a result
of using passthroughs.

* It forbids the definition of an option inside a passthrough if that option is
  also directly present in the target scope.
* It warns the developer about the options used in each passthrough, to remind
  the publisher that these options are not yet supported.
* It may also report options that might already be supported, but it should not
  prevent these options from being passed through to ``snap.yaml`` since the
  option may be defined in a way that is not yet understood by Snapcraft.

Files using in-development features will need to be updated when those features
become supported by Snapcraft.

Examples
--------

In the following example, the ``passthrough`` keyword is used at the top-level
scope to access two new, *hypothetical*, top-level features:

.. code:: yaml

   name: foo
   passthrough:
       confinement: next-generation
       planet: saturn

This example enables the usage of a new value for the existing
:ref:`snapcraft-yaml-confinement` keyword. It also enables an entirely new
``planet`` keyword to be used, along with a new value.

The following example shows the use of the keyword to access a new
application-level feature:

.. code:: yaml

   apps:
       foo:
           command: foo
           passthrough:
               daemon: complex

In this case, a passthrough is used within the scope of the app ``foo`` to set
the existing :ref:`daemon <snapcraft-yaml-daemon>` keyword to a value of
``complex``.
