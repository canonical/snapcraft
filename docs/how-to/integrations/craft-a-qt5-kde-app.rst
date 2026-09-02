.. _how-to-craft-a-qt5-kde-app:

Craft a Qt5 KDE app
===================

This how-to guide covers the steps, decisions, and implementation details that
are unique when crafting a snap for an app that uses Qt5 KDE. We'll work
through the aspects unique to Qt5 KDE apps by examining an existing project.


Example project file for KCalc
------------------------------

The following code comprises the project file of a Qt5 KDE app, `KCalc
<https://github.com/KDE/kcalc>`_. This project is the calculator of KDE Gear.

.. dropdown:: KCalc project file

    .. literalinclude:: ../code/integrations/example-qt5-kde-recipe.yaml
        :caption: snapcraft.yaml
        :language: yaml
        :lines: 2-


Add an app that uses KDE
------------------------

.. literalinclude:: ../code/integrations/example-qt5-kde-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: apps:
    :end-at: - pulseaudio

Apps that use KDE runtime libraries require the :ref:`KDE neon extension
<reference-kde-neon-extensions>`. The extension configures the runtime environment of
the app so that all desktop functionality is correctly initialized. As desktop
environment apps, they also need special configuration for AppStream and ``.desktop``
file compatibility.

To add a GTK4 app:

#. Declare the general app keys, such as ``command``,
   ``plugs``, ``after``, and so on.
#. For ``extensions``, add ``kde-neon``.
#. Set ``common-id`` to the app's unique AppStream ID. Doing so links the app
   to the ``.desktop`` launcher specified in the AppStream file.


Add a part written for Qt5 KDE
------------------------------

.. literalinclude:: ../code/integrations/example-qt5-kde-recipe.yaml
    :caption: snapcraft.yaml
    :language: yaml
    :start-at: parts:
    :end-at: - "-DKDE_SKIP_TEST_SETTINGS=ON"

Qt5 KDE parts don't require a special plugin. Instead, they need extra snap dependencies.

To add a Qt5 KDE part:

#. Declare the general part keys, such as ``plugin``, ``source``,
   ``build-packages``, and so on.
#. So that the app has access to its AppStream metadata, for ``parse-info`` add a path
   to the AppStream ``.xml`` file on the host system. Since we set ``adopt-info: kcalc``
   at the start of the project file, the AppStream file of the ``kcalc`` part will be
   used to fill in the ``summary``, ``description`` and ``icon`` of this snap and copy
   the AppStream file. See :ref:`reference-external-package-appstream` for technical
   details about how this works.
#. For ``build-snaps``, list the following dependencies:

   .. literalinclude:: ../code/integrations/example-qt5-kde-recipe.yaml
       :caption: snapcraft.yaml
       :language: yaml
       :dedent: 6
       :lines: 37-38
