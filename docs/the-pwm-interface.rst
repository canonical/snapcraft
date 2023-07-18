.. 25857.md

.. _the-pwm-interface:

The pwm interface
=================

The ``pwm`` interface allows access to a specific PWM channel on a device. The interface is restricted because it provides privileged access to PWM hardware.

Use ``snap interface pwm`` to see which PWM devices are available on the system:

.. code:: bash

   $ snap interface pwm
   name: pwm
   summary: allows access to specific PWM channel
   slots:
   - gadget:led-1
   - gadget:led-2
   - gadget:fan-1
   - gadget:fan-2
   [...]


.. _the-pwm-interface-example:

Example
-------

With an example snap application called *app* installed, the following command will connect its PWM interface to the specified PWM device channel:

.. code:: bash

   $  snap connect app:activity-led gadget:led-1

.. note::


          See :ref:`interface-management` and :ref:`supported-interfaces` for further details on how interfaces are used.


.. _the-pwm-interface-dev-details:

Developer details
-----------------

**Auto-connect**: no

**Attributes**:

* ``channel`` (slot): PWM device channel number to export and expose to consuming snaps
* ``chip-number`` (slot): chip base number to export

To use a PWM device, the snap developer must add ``plugs:[pwm]`` to a snap’s\ :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>`. The snap user can then access a specific pwm device with an :ref:`interface connection <interface-management-manual-connections>`.

Unless the snap is expected to actually use a set of PWM channels that is not predefined, it is recommended to define distinct plugs for each used pwm channel, like:

.. code:: yaml

   plugs:
     activity-led:
       interface: pwm
     warning-led:
       interface: pwm

This has the advantage of being self-documenting and 1-1 connections like these are easier to track and setup with :ref:`auto-connections <the-interface-auto-connection-mechanism>`, if the latter is needed.

When the interface is connected,

.. code:: bash

   echo (channel number) > /sys/class/pwm/pwmchipN/export

is run internally to enable access to the PWM channel.

Once connected, the consuming snap can use the device via ``/sys/class/pwm/pwmchipN/pwmX`` where *N* is the base of the PWM chip and *X* is channel number specified by the connected slot.

Finally, when the interface is disconnected,

.. code:: bash

   echo (channel number) > /sys/class/pwmchipN/unexport

is run internally to disable access to the PWM channel.

Code examples
-------------

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/pwm_test.go

The source code for the pwm interface is in the snapd repository:https://github.com/snapcore/snapd/blob/master/interfaces/builtin/pwm.go.
