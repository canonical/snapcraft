.. _how-to-debug-with-gdb:

Debug with gdb
==============

The `GNU Debugger`_ (gdb) is used by developers to introspect the execution environment
of an application, revealing both its code and data state at any point. Developers can
use gdb to debug snaps locally and remotely.


Local debugging with gdb
------------------------

Snaps are run within a :ref:`confined environment <reference-confinement>`, so running
gdb directly against a snap executable would introspect both the snapd runtime
environment and the application itself, making the identification of any issues specific
to the snap application difficult. For this reason, snapd embeds gdb within its own
framework.

To run gdb directly against an installed snap, run:

.. code-block:: bash

   snap run --gdb <snap>

When gdb is instantiated by snapd, it behaves just as it would if it was called against
the same executable outside of the snapd runtime environment.

.. note::
    For general advice on fixing potential issues in
    running snaps, see :ref:`how to debug a snap <how-to-debug-a-snap>`.
    For guidance on using gdb itself, see the `gdb documentation`_.

Generate debug symbols
----------------------

Most snaps don't ship with binaries that include debug symbols. This means gdb can’t
link to the original source. To use gdb, build and install the snap locally with debug
symbols enabled.

For example, to enable debug symbols for a typical C-based project using the
:ref:`CMake plugin <craft_parts_cmake_plugin>`, add ``set(CMAKE_BUILD_TYPE Debug)``
to the project's ``CMakeLists.txt`` then rebuild and install the snap.

After installing the snap, the symbols will be located and the source code can be
referenced, as demonstrated below:

.. terminal::

    $ snap run --gdb test-gdb.test-gdb
    [...]
    You are right before your application is execed():
    - set any options you may need
    - use 'cont' to start
    [...]
    (gdb) dir test-gdb/src/
    Source directories searched: test-gdb/src:$cdir:$cwd
    (gdb) list
    1       #include <stdio.h>
    2
    3       int main (int argc, char *argv[])
    4       {
    5         printf ("GDB from a snap is working\n");
    6
    7         return 0;
    8       }
    (gdb) cont
    Continuing.
    GDB from a snap is working
    [Inferior 1 (process 153259) exited normally]
    (gdb) quit

Local debugging with gdbserver
------------------------------

In addition to gdb, snapd integrates with `gdbserver`_ for remote access. This allows
debugging snaps with gdb using an IDE or from a remote system.

To install gdbserver on Ubuntu, run:

.. code-block:: bash

    sudo apt install gdbserver

To start gdbserver with snapd, run:

.. code-block::

    snap run --gdbserver <snap>

This enters a gdbserver shell:

.. terminal::

    $ snap run --gdbserver test-gdb
    Welcome to "snap run --gdbserver".
    You are right before your application is run.
    Please open a different terminal and run:

    gdb -ex="target remote :43041" -ex=continue -ex="signal SIGCONT"
    (gdb) continue

You can specify a port when starting the gdbserver. For example, to use port 43041, run:

.. code-block:: bash

    snap run --gdbserver=:<port> <snap>

The gdb session can now be accessed from an IDE or from gdb itself from
outside the snap:

.. terminal::

    $ gdb -ex="target remote :43041"
    GNU gdb (Ubuntu 12.1-0ubuntu1~22.04) 12.1
    [...]
    (gdb)

Remote debugging with gdbserver
-------------------------------

To debug a snap remotely, first start a gdbserver session on the remote system via ssh.
This can be done by passing the command to ssh directly:

.. code-block:: bash

    ssh <user>@<host> "sudo snap run --gdbserver=:<port> <snap>"

To connect to the remote gdbserver session, pass the remote system's IP and the port
used by gdbserver:

.. code-block:: bash

    gdb -ex="target remote <ip>:<host>"

For example, to connect to a gdbserver session on 192.168.122.138 on port 43041:

.. terminal::

    $ gdb -ex="target remote 192.168.122.138:43041"
    Welcome to `snap run --gdb`.
    You are right before your application is execed():
    - set any options you may need
    - (optionally) set a breakpoint in 'main'
    - use 'cont' to start

Remote debugging with VS Code
-----------------------------

Most IDEs support debugging snaps with gdbserver. The IDE needs to be configured to
access the snapd gdbserver session. Optionally, the IDE can be configured to
automatically start the execution of the snap with gdbserver when the debugger starts.

To remotely debug a snap with Microsoft’s `Visual Studio Code`_, you need a workspace
with the `C/C++ VS Code extension`_ installed. Additionally, you need local access to
the snap's source code and the executable. The executable can be extracted from the snap
package with:

.. code-block:: bash

    unsquashfs <snap-file>

In VS Code, open your workspace, select *Run and Debug* from the left-hand side bar,
click *create a launch.json file*, and select gdb from the list of debuggers:

.. figure:: /_static/vscode-select-gdb.png
   :width: 75%
   :align: center
   :alt: Select gdb in VS Code

This will open a template ``launch.json`` file in the editor which can be configured to
access the remote snap gdb session. Replace the contents of this file with the following
to create a new entry called *Remote gdb*:

.. code-block:: json

    {
      "version": "0.2.0",
      "configurations": [{
        "name": "Remote gdb",
        "type": "cppdbg",
        "request": "launch",
        "program": "${workspaceFolder}/squashfs-root/bin/my-app",
        "cwd": "${workspaceFolder}",

        "stopAtEntry": true,
        "stopAtConnect": true,

        "MIMode": "gdb",
        "miDebuggerPath": "/usr/bin/gdb",
        "miDebuggerServerAddress": "192.168.122.138:43041",
        "setupCommands": [{
          "description": "Enable pretty-printing for gdb",
          "text": "-enable-pretty-printing",
          "ignoreFailures": true
        }]
      }]
    }

The above example uses the target IP address and port of ``192.168.122.138:43041``. This
will need to be changed to point to your gdbserver. If gdbserver is running locally, you
can use ``localhost:<port>``. The ``program`` key needs to point at the executable in
the snap you wish to debug.

Save the file and set a breakpoint in the code before returning to the ``Run and Debug``
window. You should now see the option to run the new ``Remote gdb`` configuration:

.. figure:: /_static/vscode-run-gdb.png
   :width: 75%
   :align: center
   :alt: Run gdb in VS Code

Press the play icon to connect to the gdbserver session. You can now debug a running
snap just as you would a normally built executable.

.. figure:: /_static/vscode-gdb-console.png
   :width: 75%
   :align: center
   :alt: gdb console output in VS Code

.. _GNU Debugger: https://www.man7.org/linux/man-pages/man1/gdb.1.html
.. _gdbserver: https://www.man7.org/linux/man-pages/man1/gdbserver.1.html
.. _gdb documentation: https://sourceware.org/gdb/current/onlinedocs/gdb/
.. _Visual Studio Code: https://code.visualstudio.com
.. _C/C++ VS Code extension: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools
