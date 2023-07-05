.. 8349.md

.. _snapcraft-advanced-grammar:

Snapcraft advanced grammar
==========================

Several fields in :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` are dependent on the architecture or the operating system being exposed to Snapcraft. This is accomplished using specific and advanced syntax within the YAML consumed by the *snapcraft* command.

This *advanced grammar* is made up of three nestable statements: ``on``, ``to`` and ``try``:

The *on* statement
------------------

.. code:: yaml

       - on <selector>[,<selector>...]:
           <grammar>|<primitive>
       - else[ fail]:
           <grammar>|<primitive>

``<primitive>`` may be either a list or a scalar, depending on the keyword.

The body of the ``on`` clause is taken into account if every (AND, not OR) selector is true for the build environment. Currently the only selectors supported are host architectures (e.g. ``amd64``).

If the ``on`` clause doesn’t match and is immediately followed by an ``else`` clause, the ``else`` clause must be satisfied. An ``on`` clause without an ``else`` clause is considered satisfied even if no selector matched. The ``else fail`` form allows for the generation of an error if a ``to`` clause is not matched.

The *to* statement
------------------

.. code:: yaml

       - [on <selector>[,<selector>...] ]to <selector>[,<selector>...]:
           <grammar>|<primitive>
       - else[ fail]:
           <grammar>|<primitive>

``<primitive>`` may be either a list or a scalar, depending on the keyword.

The body of the ``to`` clause is taken into account if every (AND, not OR) selector is true for the target environment. Currently the only selectors supported are target architectures (e.g. ``armhf``).

If the ``to`` clause doesn’t match and is immediately followed by an ``else`` clause, the ``else`` clause must be satisfied. A ``to`` clause without an ``else`` clause is considered satisfied even if no selector matched. The ``else fail`` form allows for the generation of an error if a ``to`` clause is not matched.

Optionally an ‘``on``’ statement can precede a ``'to'`` in the same line to form a compound statement. Used this way, the selectors of both statements have to be true. That is to say, both the build environment and the target have to be true for the body of the clause to be taken into account.

The *try* statement
-------------------

.. code:: yaml

       - try:
           <grammar>|<primitive>
       - else:
           <grammar>|<primitive>

``<primitive>`` may be either a list or a scalar, depending on the keyword.

.. note::
          The *try* statement does not work with builds using a :ref:`base snap <base-snaps>` of ``core22`` or later.

The body of the ``try`` clause is taken into account only when all primitives contained within it are valid (primitive validity is determined on a keyword-specific basis). If they are not all valid, and are immediately followed by ``else`` clauses, those are tried in order, and one of them must be satisfied. A ``try`` clause with no ‘``else``’ clause is considered satisfied even if it contains invalid primitives.

Example
~~~~~~~

.. code:: yaml

   parts:
     _part_name_:
       stage-packages:
       # libnuma1 package isn't available in armhf architecture, make it optional
       - try:
         - libnuma1
