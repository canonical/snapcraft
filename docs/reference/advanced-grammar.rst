.. _reference-advanced-grammar:

Advanced grammar
================

Several keys in a snap's :ref:`project file <reference-snapcraft-yaml>` depend on the
architecture the snap is building on and building for. Defining different key values for
these architectures is accomplished with a specific syntax called *advanced grammar*.

This advanced grammar is made up of three statements: ``to``, ``on``, and ``try``.

The following :literalref:`build-packages <how-to-manage-dependencies>` section, for
example, evaluates and then defines which build packages to install depending on the
target environment (``to``) for the snap:

.. code-block:: yaml
    :caption: snapcraft.yaml

    build-packages:
      - to arm64:
        - g++-multilib-arm-linux-gnueabihf
        - gcc-multilib-arm-linux-gnueabihf
      - else:
        - gcc-multilib
        - g++-multilib


The ``to`` statement
--------------------

.. code-block:: yaml
    :caption: snapcraft.yaml

    - to <selector>:
        <grammar>|<value>
    - else:
        <grammar>|<value>

The body of the ``to`` clause is taken into account if every (AND, not OR) selector is
true for the target environment. The only selectors currently supported are host and
target architectures (e.g., ``amd64``).

If the ``to`` clause doesn't match and is immediately followed by an ``else`` clause,
the ``else`` clause must be satisfied. A ``to`` clause without an ``else`` clause is
considered satisfied even if no selector matched. The ``else fail`` form generates an
error if a ``to`` clause isn't matched.

An optional ``on`` statement can precede a ``to`` in the same line to form a compound
statement. Used this way, the selectors of both statements have to be true. That is to
say, both the build environment and the target have to be true for the body of the
clause to be taken into account.


The ``on`` statement
--------------------

.. code-block:: yaml
    :caption: snapcraft.yaml

    - on <selector>[,<selector>...]:
        <grammar>|<value>
    - else[fail]:
        <grammar>|<value>

The body of the ``on`` clause is taken into account if every (AND, not OR) selector is
true for the build environment. The only selectors currently supported are host and
target architectures (e.g., ``amd64``).

If the ``on`` clause doesn't match and is immediately followed by an ``else`` clause,
the ``else`` clause must be satisfied. An ``on`` clause without an ``else`` clause is
considered satisfied even if no selector matched. The ``else fail`` form generates an
error if a ``to`` clause isn't matched.


The ``try`` statement
---------------------

.. admonition:: Deprecated feature
    :class: important

    The ``try`` statement was deprecated in core20 and removed in core22.

.. code-block:: yaml
    :caption: snapcraft.yaml

    - try:
        <grammar>|<value>
    - else:
        <grammar>|<value>

The body of the ``try`` clause is taken into account only when all provided values are
valid. The validity of values is determined on a keyword-specific basis. If they're not
all valid, any subsequent ``else`` clauses are tried in order, and one of them must be
satisfied. A ``try`` clause with no ``else`` clause is considered satisfied even if it
contains invalid values.


Examples
--------

The following examples will set different environment variables for the build stage,
depending on the host (``on``) and target (``to``) architectures:

.. code-block:: yaml
    :caption: snapcraft.yaml

    build-environment:
      - on amd64 to arm64:
        - FOO: BAR
      - on amd64 to armhf:
        - FOO: BAZ

.. code-block:: yaml
    :caption: snapcraft.yaml

    build-environment:
      - on amd64 to arm64:
        - FOO: BAR
      - on amd64 to armhf:
        - FOO: BAZ
