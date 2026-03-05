
Dependencies
------------

Whether the Python interpreter needs to be included in the snap depends on its
base and confinement.

core26
~~~~~~

- The core26 base snap doesn't include Python, so the snap must always bundle it. This
  is typically done with the :ref:`stage-packages <PartSpec.stage_packages>` key.

core24 and lower
~~~~~~~~~~~~~~~~

- Projects with ``strict`` or ``devmode`` confinement can safely use the base
  snap's interpreter, so they typically do **not** need to include Python.
- Projects with ``classic`` confinement **cannot** use the base snap's
  interpreter and thus must always bundle it. This is typically done with the
  ``stage-packages`` key.
- In both cases, a specific/custom Python installation can always be included
  in the snap. This can be useful, for example, when using a different Python
  version or building an interpreter with custom flags.

Snapcraft will prefer an included interpreter over the base's, even for projects
with ``strict`` and ``devmode`` confinement.
