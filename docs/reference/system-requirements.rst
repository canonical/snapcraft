.. _reference-system-requirements:

System requirements
===================

To run Snapcraft, a system requires the following minimum hardware and
installed software. These requirements apply to local hosts as well as VMs and
container hosts.


Minimum hardware requirements
-----------------------------

- AMD64, ARM64, ARMv7-M, RISC-V 64-bit, PowerPC 64-bit little-endian, or S390x
  processor
- 2GB RAM
- 10GB available storage space
- Internet access for remote software sources and the Snap Store


Platform requirements
---------------------

.. list-table::
  :header-rows: 1
  :widths: 1 3 3

  * - Platform
    - Version
    - Software requirements
  * - GNU/Linux
    - Popular distributions that ship with systemd and are `compatible with
      snapd <https://snapcraft.io/docs/installing-snapd>`_
    - systemd

      snapd 2.44.3 or higher
  * - Windows
    - Windows 10 version 2004 or higher
    - `Windows Subsystem for Linux 2 (WSL2) <https://ubuntu.com/desktop/wsl>`_
  * - macOS
    - macOS Sonoma or higher
    - `Homebrew <https://brew.sh>`_
