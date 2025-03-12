.. _how-to-debug-a-snap:

Debug a snap
============


Recommended debugging workflow
------------------------------


Debugging command cheat sheet
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. table of commands goes here


Where to look for errors
------------------------


Build environment
~~~~~~~~~~~~~~~~~


Missing libraries
~~~~~~~~~~~~~~~~~


Staged packages
^^^^^^^^^^^^^^^


Individual libraries
^^^^^^^^^^^^^^^^^^^^


Interfaces
~~~~~~~~~~


Check for runtime errors
------------------------

.. --devmode



Open a shell into the build container
-------------------------------------

.. --shell


Iterate over the build lifecycle
--------------------------------


Before the build
~~~~~~~~~~~~~~~~

.. snapcraft.yaml


Build step
~~~~~~~~~~


Stage step
~~~~~~~~~~


Trace system calls
------------------

.. --strace


Check file permissions
----------------------

.. device cgroups


Check for policy violations
---------------------------

.. snappy-debug


Extract policy violation logs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Examine AppArmor violations
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Examine seccomp violations
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. include info about snap-seccomp versions and paths
