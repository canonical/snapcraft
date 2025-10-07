.. _release-8.13:

Snapcraft 8.13 release notes
============================

.. add date here, once scheduled

Learn about the new features, changes, and fixes introduced in Snapcraft 8.13.


Requirements and compatibility
------------------------------
See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


What's new
----------

Snapcraft 8.13 brings the following features, integrations, and improvements.


Set component versions dynamically
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Previously, a component could only set its version statically, when first declared.

Parts can now set a component versions dynamically. If a component points to a part with
the ``adopt-info`` key, the part can call craftctl to set the version.

For detailed guidance, see
:ref:`how-to-access-project-variables-across-parts-and-components`.


Documentation submodule name change
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Git submodule containing documentation components has been renamed to
``sphinx-docs-starter-pack`` to match its parent repository.

If you're a returning contributor to the project, after you pull the latest commits, run
the following commands in your local repository to sync the submodule change:

.. code-block::

    git submodule sync
    git submodule update --init --recursive
    git clean -ffd
