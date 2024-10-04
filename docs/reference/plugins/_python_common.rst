
Dependencies
------------

Whether the Python interpreter needs to be included in the snap depends on its
``confinement``. Specifically:

- Projects with ``strict`` or ``devmode`` confinement can safely use the base
  snap's interpreter, so they typically do **not** need to include Python.
- Projects with ``classic`` confinement **cannot** use the base snap's
  interpreter and thus must always bundle it (typically via ``stage-packages``).
- In both cases, a specific/custom Python installation can always be included
  in the snap. This can be useful, for example, when using a different Python
  version or building an interpreter with custom flags.

Snapcraft will prefer an included interpreter over the base's, even for projects
with ``strict`` and ``devmode`` confinement.
