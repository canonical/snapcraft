.. 26503.md

.. _the-kernel-crypto-api-interface:

The kernel-crypto-api interface
===============================

The ``kernel-crypto-api`` interface allows access to the `Linux kernel crypto API <https://www.kernel.org/doc/html/v4.11/crypto/index.html>`__, which itself provides a set of cryptographic ciphers and other data transformation mechanisms.

.. note::

   See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.


Developer details
-----------------

**Auto-connect**: no

The kernel crypto API has been designed to be used by any process such that using it requires no special privileges. As this provides a kernel surface, and has a CVE history, this interface needs to be manually connected.

Code examples
-------------

The test code can be found in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/kernel_crypto_api_test.go

The source code for the interface is in the snapd repository: https://github.com/snapcore/snapd/blob/master/interfaces/builtin/kernel_crypto_api.go
