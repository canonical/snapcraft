.. 26505.md

.. _the-microstack-support-interface:

The microstack-support interface
================================

The ``microstack-support`` interface enables multiple service access for the *Microstack infrastructure*. It’s used by the `Microstack <https://microstack.run/>`__ snap, a full OpenStack deployment within a single snap package.

Virtual machines are spawned as QEMU processes with libvirt acting as a management daemon (including for activities such as applying AppArmor profiles).

Networking is provided largely via OpenVSwitch and Neutron, with dnsmasq acting as an auxiliary daemon. A tun/tap kernel module is used for creating virtual interfaces.

Virtual machines rely on KVM for virtualisation acceleration and the vhost framework in the kernel (vhost_net, vhost_scsi, vhost_vsock).

.. note::

          See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.

--------------


.. _the-microstack-support-interface-dev-details:

Developer details
-----------------

:ref:`Auto-connect <interface-management-auto-connections>`: no

:ref:`Super-privileged <super-privileged-interfaces>`: yes

This interface allows MicroStack to operate by allowing the necessary system calls to be used by the following services:

- libvirt
- qemu
- qemu-img
- Nova
- Neutron
- Keystone
- Glance
- Cinder

Code examples
-------------

The snapcraft.yaml for MicroStack can be found here: https://github.com/coreycb/microstack/blob/1-tls/snapcraft.yaml

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/microstack_support.go
