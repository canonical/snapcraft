Removing the snap
-----------------

Removing the snap is simple too:

.. code-par:: bash

   $  sudo snap remove |execname|

You can clean up the build environment with the following command:

.. code:: bash

   $ snapcraft clean

When you make a change to :file:`snapcraft.yaml` and rebuild the snap,
Snapcraft only builds the parts that have changed. Cleaning a build, however, forces your snap to be rebuilt in a clean environment and will take longer.

