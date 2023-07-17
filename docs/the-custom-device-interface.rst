.. 29487.md

.. _the-custom-device-interface:

The custom-device interface
===========================

The ``custom-device`` interface permits access to a device of a specific class and model without requiring the creation of an interface for that device alone. It’s intended to be used with :term:`Ubuntu Core` and its scope and specification are defined as part of the :ref:`gadget snap <gadget-snaps>` for the deployed Ubuntu Core image.

The slot-side of the interface is used to derive which *udev* rules are provided to the plug-side of the connection:

.. code:: yaml

   slots:
     dual-sd:
       interface: custom-device
       custom-device: my-dual-sd-device
       devices:
         - /dev/DualSD

To prevent connection to arbitrary custom-device slots, the plug and slot must share the same custom-device attributes, including the name of the plug or slot:

.. code:: yaml

   plugs:
     dual-sd:
       interface: custom-device
       custom-device: my-dual-sd-device
   apps:
     app:
       plugs: [dual-sd]

When the slot and plug are connected, a udev rule is automatically generated and tagged for the plug side for each device path in the ``devices`` and ``read-devices`` attributes, such as:

::

   KERNEL=="DualSD",

Note that here, the ``KERNEL`` specification is the basename of the full device path. For this reason, the interface requires that all device paths listed in ``devices`` and ``read-devices`` must have unique basenames and must begin with ``/dev/``.

If the ``udev-tagging`` attribute is used, this default udev rule is replaced with more specific rules, as described below.

Requires snapd version *2.55+*.

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _`the-custom-device-interface-dev-details`:

Developer details
-----------------

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: yes

**Attributes**:

* ``custom-device`` (plug, slot): label for the custom device. Needs to be identical across the plug and slot connections.
* ``devices`` (slot): path to device node. Example: ``devices: [/dev/input/event[0-9], /dev/input/mice]``
* ``files`` (slot):
* ``read`` (slot): list of files and/or directories for read-only access by the device. Example: ``read: [ /dev/input/by-id/* ]``
* ``write`` (slot): list of files and/or directories for read/write access by the device. Example: ``write: [ /etc/file-write, /etc/dir-write ]``
* ``udev-tagging`` (optional): used to tailor the generated udev rules. Can be one of the following:
* ``kernel``: (mandatory): maps to the string used as the udev ``KERNEL==`` filter rule.
* ``subsystem``: corresponds to the ``SUBSYSTEM==`` filters in a udev rule.
* ``environment``: a map of expected environment variables for the udev rule to match with ``ENV{...}=="..."``
* ``attributes``: a map of attributes used with ``ATTR{...}=="..."``

Code examples
-------------

A truncated example showing how the subsystem and attributes can be used:

.. code:: yaml

      udev-tagging:
        - kernel: hiddev0
          subsystem: usb
          attributes:
            idVendor: "0x03f0" # HP
        - kernel: hiddev1
          subsystem: usb
          attributes:
            idVendor: "0x03fc" # ECS

An example slot declaration showing the how the kernel environment settings can be used with a custom joystick interface:

.. code:: yaml

   slots:
     hwdev:
       interface: custom-device
       custom-device: custom-joystick
       devices:
         - /dev/input/js{[0-9],[12][0-9],3[01]}
         - /dev/input/event[0-9]*
       files:
         read:
           - /run/udev/data/c13:{6[5-9],[7-9][0-9],[1-9][0-9][0-9]*}
           - /run/udev/data/c13:{[0-9],[12][0-9],3[01]}
           - /sys/devices/**/input[0-9]*/capabilities/*
       udev-tagging:
         - kernel: event[0-9]*
           subsystem: input
           environment:
             ID_INPUT_JOYSTICK: "1"

The above example will generate the following udev tags:

::

   spec.TagDevice(`KERNEL=="js{[0-9],[12][0-9],3[01]}"`)
   spec.TagDevice(`SUBSYSTEM=="input", KERNEL=="event[0-9]*", ENV{ID_INPUT_JOYSTICK}=="1"`)

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/custom_device_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/custom_device.go
