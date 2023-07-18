.. 33649.md

.. _classic-confinement:

Classic confinement
===================

Classic confinement is a permissive :ref:`Snap confinement <snap-confinement>` level, equivalent to the full system access that traditionally packaged applications have.

It’s often used as a stop-gap measure to enable developers to publish applications that need more access than the current set of interfaces and permissions allow.

This document serves as a reference for software developers who intend or need to build their snaps as classic. It outlines the principles and implementation of the classic confinement in snaps. It provides explanations and examples on what happens at build, install and runtime for snaps packaged using classic confinement.


Confinement levels
------------------

Security confinement distinguishes snaps from software distributed using the traditional repository methods.

The confinement mechanism allows for a high level of isolation and security, and prevents snaps from being affected by underlying system changes, one snap affecting another, or snaps affecting the host system. `Security policy and sandboxing <https://forum.snapcraft.io/t/554>`__ details how confinement is implemented.

Different confinement levels describe what type of access snap applications have once installed on the user’s system. Confinement levels can be treated as filters that define what type of system resources the application can access outside the snap.

Confinement is defined by general levels and fine-tuned using interfaces, and there are three levels of confinement; **strict**, **classic** and **devmode**.

-  **Strict**\  This confinement level uses Linux kernel security features to lock down the applications inside the snap. By default, a strictly confined application cannot access the network, the users’ home directory, any audio subsystems or webcams, and it cannot display any graphical output via X or Wayland.

-  **Devmode**\  This is a debug mode level used by developers as they iterate on the creation of their snap. With devmode, applications can access resources that would be blocked under strict confinement. However, the access to these resources will be logged, so the developers can then review the software behavior and add interfaces as required. This allows developers to troubleshoot applications, because they may behave differently when confined.

-  **Classic** This is a permissive level equivalent to the full system access that traditionally packaged applications have.

   Classic confinement is often used as a stop-gap measure to enable developers to publish applications that need more access than the current set of permissions allow. The classic level should be used only when required for functionality, as it lowers the security of the application. Examples of classic snaps would include development environments, terminals or build tools that need to access or execute arbitrary files on the host system.

   Classically confined snaps are reviewed by the Snap Store reviewers team before they can be published. Snaps that use classic confinement may be rejected if they don’t meet the necessary requirements.


Inside classic confinement
--------------------------

Applications can be packaged as classic snaps for a variety of reasons. Primarily, this level of confinement is used for applications that need access to arbitrary binaries on the system, which is not possible when using strict confinement.

Applications packaged as classic snaps then behave *almost* like software provided and installed through the system’s repository archives, using the traditional packaging mechanisms (like apt or rpm), but with some important distinctions.


Classic confinement at runtime
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When a classic snap is executed on the host system, the snap daemon, *snapd*, will perform the following actions:

.. figure:: https://assets.ubuntu.com/v1/d4018ec4-confinement_01.png
   :alt: Snap confinement at run time


**Snap daemon (snapd) actions at run time**

1. snapd runs a sub-process called ``snap-confine``, which is responsible for creating the necessary confinement for the snap (with the rules set during the installation process).
2. The `AppArmor <https://ubuntu.com/server/docs/security-apparmor>`__ profile will be loaded in complain mode. This can be ascertained by running ``apparmor_status`` and find the relevant profiles listed in the complain section of the output, e.g.: ``snap.XXX.hook.configure`` and ``snap.XXX.snapcraft``
3. For classic snaps, the permissive Seccomp profile will be parsed. This can be ascertained by running ``grep Seccomp /proc/PID/status``, with the value of ``0`` indicating the process does not currently use any Seccomp filtering.
4. Load the dynamic library dependencies in the following order: ``/snap/<base>`` (with the relevant base specified in snap.yaml derived from the developer edited snapcraft.yaml).

   -  If not found, search through the host system using the system’s /etc/ld.so.cache. Classic snaps should not have a ``$LD_LIBRARY_PATH`` configured as part of their runtime environment.

+-------------------------+-----------------------+--------------------------------+
|                         | **Strict**            | **Classic**                    |
+=========================+=======================+================================+
| **Mount namespace**     | private               | none                           |
+-------------------------+-----------------------+--------------------------------+
| **cgroups**             | Yes                   | No                             |
+-------------------------+-----------------------+--------------------------------+
| **AppArmor**            | Enforce mode          | Complain mode                  |
+-------------------------+-----------------------+--------------------------------+
| **Seccomp**             | Strict filtering      | No filtering                   |
+-------------------------+-----------------------+--------------------------------+
| **LD_LIBRARY_PATH**     | Depends               | Empty                          |
+-------------------------+-----------------------+--------------------------------+
| **Library loading**     | Staged packagesBase   | Staged packagesBaseHost system |
+-------------------------+-----------------------+--------------------------------+


Possible conflicts
^^^^^^^^^^^^^^^^^^

Since there is no isolation between classic snaps and the underlying host system, at runtime, classic snaps may load dynamic library dependencies in a way that could create a possible error or conflict, leading to application instability, unknown behavior or crash.

A classic snap created with Snapcraft using one of the Ubuntu bases with dynamically linked binaries will try to load the required dependencies at runtime.

-  It will try to load the dependencies, including stage packages and any other libraries inside the snap.
-  If not found, it will try to find the dependencies in the base snap under ``/snap/<base>``, where base can be something like ``core20``, ``core22``, etc. The libraries will need to match the name and version of libraries as provided by the Ubuntu repository archives for the specific base, e.g.: snaps built with ``core20`` will need to use the relevant libraries (by name or version) the way they are defined for Ubuntu 20.04 LTS.
-  If not found, it will try to find the dependencies on the host system.
-  If found, the libraries will be used.
-  The loaded host libraries may not match the expected snap/core version, which could result in application instability, unknown behaviour or crash.

Pre-built binaries
------------------

Since there is no isolation between classic snaps and the underlying host system, special care needs to be taken care of any pre-built binaries with hard-coded library dependency paths, as they will “skip” the normal loading order of libraries at runtime.

This is outlined in the :ref:`build time <classic-confinement-build>` section
below.


Classic confinement at install time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When a classic snap is installed, *snapd* will perform the following actions:

.. figure:: https://assets.ubuntu.com/v1/35306066-confinement_02.png
   :alt: Snap confinement at install time


1. Mount the snap as a loopback device.
2. Skip the creation of the snap-specific private mount namespace.
3. Skip the configuration of the device cgroups.
4. Create a permissive AppArmor profile (which will be loaded in complain mode at runtime). The profile is stored under ``/var/lib/snapd/apparmor/profiles``.
5. Create a permissive Seccomp profile (which will be parsed at runtime). The profile is stored under ``/var/lib/snapd/seccomp/bpf`` and will contain the following entry:

   -  ``@unrestricted\n``


.. _classic-confinement-build:

Classic confinement at build time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

:ref:`Snapcraft <snapcraft-overview>` builds classic snaps differently from snaps with strict confinement.

.. figure:: https://assets.ubuntu.com/v1/24ce3093-confinement_03.png
   :alt: Snap confinement at run time


This is because, in order to execute correctly, classic confined snap packages require dynamic executables to load shared libraries from the appropriate base snap instead of using the host’s root filesystem.

To prevent incompatibilities, binaries in classic snaps must be built with appropriate linker parameters, or patched to allow loading shared libraries from their base snap. In case of potential dynamic linking issues, the snap author must be aware that their package may not run as expected.

There are multiple ways dynamic linking parameters can be manipulated:

-  **Runtime library paths**\  The dynamic section of an ELF file contains the RPATH entry, which lists the runtime paths to shared libraries to be searched before the paths set in the LD_LIBRARY_PATH environment variable. Multiple paths separated by a colon can be specified.
-  **$ORIGIN path**\  The special value ``$ORIGIN`` represents the path where the binary is located, thus allowing the runtime library path to be set relative to that location (e.g.: ``$ORIGIN/../lib`` for an executable installed under ``bin/`` with libraries in ``lib/``).
-  **File interpreter** The special ELF section *.interp* holds the path to the program interpreter. If used, it must be set to the path of the appropriate dynamic linker - the dynamic linker from the snap package being created If libc is staged, or the dynamic linker provided by the base snap otherwise. Usually, the program interpreter is provided by the base, but it can also be provided by the snap. This happens before any library resolution takes place.

To execute as expected, binaries in a classic snap application must be configured to look for shared libraries provided by the base snap or bundled as part of the application snap. This is achieved by setting the runtime path to shared libraries in all ELF binaries (except relocatable object files) that are present in the package payload.

-  The ``$RPATH`` value must be set to reach all **needed** entries in the dynamic section of the ELF binary.
-  If the binary already contains an ``$RPATH``, only those that mention ``$ORIGIN`` are kept.
-  ``$RPATH`` entries that point to locations inside the payload are changed to be relative to ``$ORIGIN``.


Setting RPATH from sources (using *Snapcraft*)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

An ELF binary created during the parts lifecycle execution can have its ``RPATH`` value set by using appropriate linker parameters. The linker is typically invoked indirectly via a compiler driver; in the *gcc* case parameters can be passed to the linker using the ``-Wl`` option:

.. code:: bash

   gcc -o foo foo.o -Wl,-rpath=\$ORIGIN/lib,--disable-new-dtags -Llib -lbar


Setting RPATH for pre-built (ELF) binaries - Patching generated executables
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Snaps may contain pre-built ELF binaries installed from arbitrary sources (typically from the distribution repository archives, after installing stage packages). In this case ``RPATH`` must be set by modifying the existing binary using a tool such as `PatchELF <https://snapcraft.io/install/patchelf/>`__:

.. code:: bash

   patchelf --force-rpath --set-rpath \$ORIGIN/lib “binary file”

PatchELF can also be used to change the interpreter to a different dynamic linker:

.. code:: bash

   patchelf --set-interpreter /lib64/ld-linux-x86-64.so.2 foo


Possible conflicts
~~~~~~~~~~~~~~~~~~

Patching ELF binaries to modify ``RPATH`` or interpreter entries may fail in certain cases, as with binaries using libc variants that require a nonstandard interpreter. Additionally, patching will cause signed binaries to change the signature of the binaries, which may have the side effect of failed validation for tools or scenarios where the software hashes were generated beforehand.
