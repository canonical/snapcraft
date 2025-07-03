.. _explanation-command-chain:

Command Chain
=============

Command chain is a mechanism for running small pieces of code before the main
application executable. The chain uses ``exec`` to transition the process from
the current element of the chain to the next. The last element of the chain is
always the application executable.

Uses
----

Command chain may be used to modify process environment, perform initialization
steps or anything else that makes sense in a given situation. Command chain
elements are usually reusable across applications. Command chain elements are
also usually stackable, so an application can use several tailored command
chain elements to perform unrelated functions.

.. code-block:: yaml
   :caption: snapcraft.yaml

   apps:
     hello:
       command: bin/hello
       command-chain:
       - bin/foo
       - bin/bar

Here, when the application ``hello`` is started, snapd performs a sequence of
initialization steps executed by distinct programs. Execution starts by running
``/bin/snap``. Execution then continues by ``exec`` to a specific copy of
``snap-confine``, followed by another ``exec`` to ``snap-exec``. In the absence
of command-chain elements, ``snap-exec`` executes the application binary. In
the presence of command chain, the first element of the chain, ``bin/foo``
executes instead.

Protocol
--------

Every element of the chain is responsible for executing the next chain by
calling ``exec`` (without forking the process).  Here both ``bin/foo`` and
``bin/bar`` would follow the same protocol, ultimately executing ``bin/hello``.
The next element of the chain is passed through the process argument vector.

In the example above, ``snap-exec`` would call ``exec bin/foo bin/bar
bin/hello`` with ``bin/foo`` as the program path and program name and ``bin/bar
bin/hello`` as arguments. This protocol continues until the application entry
point is reached.

Example
-------

Both ``bin/foo`` and ``bin/bar`` can be written as simple shell scripts or
comprehensive, compiled programs. A shell version might look like this.  Notice
that the argument ``$0`` is the name of the program (command chain element),
while remaining arguments, here represented as ``"$@"`` are executed to
implement the protocol.

.. code-block:: shell
   :caption: bin/foo

   #!/bin/sh
   echo "command chain element $0"
   exec "$@"


The line ``exec "$@"`` can be written more verbosely to illustrate how chain
elements link together via ``exec``.

.. code-block:: shell
   :caption: bin/foo

   #!/bin/sh
   echo "command chain element $0"
   chain_next="$1"
   shift
   exec "$chain_next" "$@"
