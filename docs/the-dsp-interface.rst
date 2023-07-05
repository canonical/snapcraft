.. 25491.md

.. _the-dsp-interface:

The dsp interface
=================

The ``dsp`` interface allows for the control of digital signal processors (DSPs) on specific devices and systems (such as specific *Ambarella* devices)

.. note::


          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-dsp-interface-dev-details:

Developer details
-----------------

**Auto-connect**: no

This interface allows privileged access to hardware and kernel drivers related to the digital signal processor and thus is only allowed on specific devices providing the slot via a gadget and is also not auto-connected.

Requires snapd version *2.51+*.


.. _the-dsp-interface-code:

Code examples
~~~~~~~~~~~~~

The source code for this interface is in the *snapd* repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/dsp.go
