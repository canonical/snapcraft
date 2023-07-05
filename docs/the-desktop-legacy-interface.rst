.. 7782.md

.. _the-desktop-legacy-interface:

The desktop-legacy interface
============================

``desktop-legacy`` allows privileged access to desktop legacy methods.

**Auto-connect**: yes

**Transitional**: yes

Requires snapd version *2.28+*.

See :ref:`The desktop interfaces <the-desktop-interfaces>` for further details.

**Condition to Connect**:

* If your snap has a graphical interface, you *should* connect to this interface.
* If your snap is expected to receive text input from CJKV(Chinese, Japanese, Korean, Vietnamese etc.) users, you **must** connect to this interface, *failing to do so will cause frustration to these users*.

**Compatibility Information**:

* Remote parts provided by `ubuntu/snapcraft-desktop-helpers: Various launchers for snapcraft wiki parts <https://github.com/ubuntu/snapcraft-desktop-helpers>`__ must be adopted by the snap in order to make input method support work
* Currently snapd is compatible with iBus and Fcitx input method framework, Fcitx support is currently limited due to a bug in ubuntu/snapcraft-desktop-helpers - `link <https://github.com/ubuntu/snapcraft-desktop-helpers/pull/156>`__

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
