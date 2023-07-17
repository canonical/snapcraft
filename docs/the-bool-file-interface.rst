.. 7803.md

.. _the-bool-file-interface:

The bool-file interface
=======================

The ``bool-file`` interface allows access to a specific class of file that contains boolean semantics, typically used to toggle or represent the state of binary hardware values.

This interface is primarily intended to be used with :term:`Ubuntu Core` devices, it’s also restricted because it provides privileged access to hardware.

These kinds of file are located within specific directories inside the `sysfs <https://man7.org/linux/man-pages/man5/sysfs.5.html>`__ filesystem (``/sys``) and this interface allows a file to be *read*, to obtaining a current value, or *written to*, setting a new value.

The `LED class <https://www.kernel.org/doc/html/latest/leds/leds-class.html>`__ is a good example of file type for this interface, as it potentially allows LEDs to be turned on and off, and have their brightness modified.

Use the ``snap interface bool-file`` command to see which boolean files are available on the system:

.. code:: bash

   $ snap interface bool-file
   name:    bool-file
   summary: allows access to specific file with bool semantics
   slots:
     - my-gadget:green-led
     - my-gadget:red-led
   [...]

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-bool-file-interface-dev-details:

Developer details
-----------------

:ref:`Auto-connect <interface-management-auto-connections>`: no

**Attributes**:

* ``path`` (slot): path to the file in *sysfs*

Example: ``/sys/class/leds/green/brightness``

The path value must match one of the following regular expressions:

- For GPIO devices: ``^/sys/class/gpio/gpio[0-9]+/value$``
- For LED devices: ``^/sys/class/leds/[^/]+/brightness$``

The :ref:`gpio interface <the-gpio-interface>` provides another option for accessing GPIO devices.

To use a boolean file, the snap developer must add ``plugs: [ bool-file ]`` to a snap’s :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>`. The snap user can then access a specific boolean file with an :ref:`interface connection <interface-management-manual-connections>`.

Unless a snap specifically expects a set of boolean files that cannot be predefined, the recommended approach is to define distinct plugs for each boolean file the snap wishes to use:

.. code:: yaml

   plugs:
     green-led:
       interface: bool-file
     red-led:
       interface: bool-file

Defining distinct plugs for each boolean file has the advantage of being self-documenting, and 1:1 connections like these are easier to track and setup with :ref:`auto-connections <the-interface-auto-connection-mechanism>`, if needed.

Once connected, the consuming snap can use the boolean file via the path mentioned in the ``path`` attribute specified by the connected slot.

The slot side on a gadget snap may be declared as follows:

.. code:: yaml

   slots:
     green-led:
       interface: bool-file
       path: /sys/class/leds/green0/brightness
     red-led:
       interface: bool-file
       path: /sys/class/leds/red0/brightness

Code examples
-------------

The test code for this interface can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/bool_file_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/bool_file.go

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
