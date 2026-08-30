.. _how-to-debugging:

Debugging
=========

Like all software, snaps need to be debugged sooner or later. Debugging is a broad and
difficult activity, so these guides focus on the specific debugging and linting features
that Snapcraft provides.

While Snapcraft shares debugging methods and practices with software development, there
are some patterns to look out for with snaps.

- :ref:`how-to-debug-a-snap`
- :ref:`how-to-debug-classic-confinement`

Each linter in Snapcraft checks for different categories of problems, and has specific
remedies.

- :ref:`how-to-use-the-classic-linter`
- :ref:`how-to-use-the-library-linter`
- :ref:`how-to-use-the-metadata-linter`
- :ref:`how-to-use-the-gpu-linter`
- :ref:`how-to-disable-a-linter`

With especially difficult problems in snaps, you can use GDB to debug the build
environment of the snap.

- :ref:`how-to-debug-with-gdb`


.. toctree::
    :hidden:

    debug-a-snap
    use-the-classic-linter
    use-the-library-linter
    use-the-metadata-linter
    use-the-gpu-linter
    disable-a-linter
    debug-with-gdb
    Classic confinement <debug-classic-confinement>
