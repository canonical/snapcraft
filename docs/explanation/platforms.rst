.. meta::
    :description: Learn about platforms and how Snapcraft uses platform and architecture keys to define snap builds for different CPU architectures.

.. _explanation-platforms:

Platforms
=========

All software is constrained to a compatible CPU architecture. It may be built natively
on the same architecture as the host it runs on, or cross-compiled to be built on a host
with one architecture and run on a host with another.

In Snapcraft, the platform describes the combination of hardware and software
constraints for the snap. It signifies what hardware and what applications the snap
is intended for. A platform named ``amd64`` would be used for typical installations on
AMD64, while a second platform named ``amd64-debug`` would include binaries built with
debug flags enabled for developers.

Aside from a name, the platform declares the CPU architectures that the snap builds and
runs on as pairs of ``build-on`` and ``build-for`` keys.

Snaps based on core22 implement platforms differently. They limit a snap
to one pair of architectures, and instead use the :ref:`architectures
<reference-snapcraft-yaml-architectures>` key.

.. _build-plans:

Build plans
-----------

Snapcraft uses the :ref:`platforms <reference-snapcraft-yaml-platform-keys>` and
:ref:`architectures <reference-snapcraft-yaml-architectures>` keys to create a *build
plan*, which is the set of all snaps it can build. Snapcraft generates the build plan
and iterates through it to create a snap for each platform. You can filter this build
plan using command-line arguments and environment variables.

The happy path of a build plan
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Consider the following platforms entry:

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      platform1:
        build-on: [amd64]
        build-for: [amd64]
      platform2:
        build-on: [amd64, riscv64]
        build-for: [riscv64]


When Snapcraft starts, it creates a full build plan:

* platform: platform1, build-on: AMD64, build-for: AMD64
* platform: platform2, build-on: AMD64, build-for: RISCV64
* platform: platform2, build-on: RISCV64, build-for: RISCV64

Snapcraft then filters the build plan to entries where ``build-on`` matches the host's
architecture. On an AMD64 system, the filtered plan is:

* platform: platform1, build-on: AMD64, build-for: AMD64
* platform: platform2, build-on: AMD64, build-for: RISCV64

With this filtered plan, Snapcraft builds two snaps: one that runs on AMD64 and one that
runs on RISCV64.

You can filter the build plan further. Using the command-line argument ``--platform
riscv64`` narrows the plan to:

* platform: platform2, build-on: AMD64, build-for: RISCV64

Build isolation
~~~~~~~~~~~~~~~

When a project defines multiple platforms, Snapcraft will produce snaps for
different host and architecture combinations.  Each of these builds may require
architecture-specific repositories, toolchains, or system libraries.

These changes to the build environment can conflict with builds targeting a different
architecture. Snapcraft would need to clean part data between each build to prevent
cross-contamination, which slows iterative development.

To avoid these issues, Snapcraft isolates each build in its own environment.

In destructive mode, all builds occur directly on the host. To avoid contamination,
Snapcraft builds only one snap in this mode. If the build plan contains multiple
snaps, Snapcraft fails and requires you to filter the plan to a single item. The
:ref:`platforms reference<reference_platforms_environment_variables>` describes how to
filter the build plan.

Building on Launchpad
~~~~~~~~~~~~~~~~~~~~~

You can build snaps on Launchpad using the :ref:`ref_commands_remote-build` command, on
Launchpad.com, or through the Snap Store.

Build behavior differs on Launchpad compared to local builds. When building locally,
you're constrained to entries where ``build-on`` matches your host's architecture.
Launchpad has build environments for many architectures, so it can build on multiple
architectures for the same project.

Launchpad uses additional heuristics to decide where to build snaps. For example,
consider this platforms definition:

.. code-block:: yaml
    :caption: snapcraft.yaml

    platforms:
      ppc64el:
        build-on: [amd64, riscv64]
        build-for: [ppc64el]

Launchpad may build the snap on either AMD64 or RISCV64. This choice is controlled by
Launchpad and cannot be influenced.
