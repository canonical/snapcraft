.. 28298.md

.. _the-kernel-module-load-interface:

The kernel-module-load interface
================================

``kernel-module-load`` provides the ability to load, or deny loading, specific kernel modules. This interface gives privileged access to the device.

See also the :ref:`kernel-module-control interface <the-kernel-module-control-interface>` for inserting, removing and querying kernel modules.

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-kernel-module-load-interface-dev-details:

Developer details
-----------------

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: yes

**Attributes**:

* ``name`` (plug, required): provides the name of the kernel module to be loaded (eg, ‘``name: pcspkr``’)
* ``load`` (plug): string to declare when, or whether, to load this module. Values can be one of the following:

  - ``on-boot`` (default): loads the kernel module at boot time (eg. ``load: on-boot``)
  - ``denied``: prevents the module from being loaded at all (eg. ``load: denied``). Also known as *denylisting* in the Linux kernel.

* ``options`` (plug): string of options to use when loading the module (eg, ``options: p1=3 p2=true p3``)

**In addition to the ``name`` attribute being required, either ``options`` or ``load`` must also be specified.**

Consumers of this interface require a `snap declaration <https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`__ for distribution via the Snap Store.

Requires snapd version *2.54+*.


.. _the-kernel-module-load-interface-code:

Code examples
~~~~~~~~~~~~~

The following is an example snippet for an application snap to load the module ``foo`` with options ``param=2``, and to deny loading the module ``bar``:

.. code:: yaml

   plugs:
     load-foo:
       interface: kernel-module-load
       modules:
       - name: foo
         options: param=2
     deny-bar:
       interface: kernel-module-load
       modules:
       - name: bar
         load: denied

The ``load-foo`` *foo* kernel module can be more verbosely by declared with the ``load`` attribute, resulting in the same behaviour:

.. code:: yaml

   plugs:
     load-foo:
       interface: kernel-module-load
       modules:
       - name: foo
         load: on-boot
         options: param=2

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/kernel_module_load.go

The source code for the interface is in the snapd repository:https://github.com/snapcore/snapd/blob/master/interfaces/builtin/kernel_module_load_test.go
