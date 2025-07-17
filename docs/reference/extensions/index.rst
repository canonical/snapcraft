.. _reference-extensions:

Extensions
==========

Snapcraft extensions enable you to easily incorporate a set of common requirements into
a snap, saving time spent replicating the same general requirements shared across
similar apps.

Extensions instruct Snapcraft to operate on the project file prior to build, causing it
to add any needed scaffolding and boilerplate keys to enable a particular technology.
The procedure is merely a postprocessor acting on the project's keys in memory -- the
actual project file on disk is unaffected.

For guidance on specific extensions, see :ref:`how-to-extensions`.

.. toctree::
    :hidden:

    env-injector-extension
    gnome-extension
    kde-neon-extensions
    flutter-extension
    ros-1-extension
    ros-1-content-extensions
    ros-2-extensions
    ros-2-content-extensions
