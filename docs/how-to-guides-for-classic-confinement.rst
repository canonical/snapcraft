.. 34416.md

.. _how-to-guides-for-classic-confinement:

How to guides for classic confinement
=====================================

:ref:`Classic confinement <snap-confinement>` is used for snaps that need to access system resources outside the strict confinement usually used.

When a snap uses classic confinement, it relies on libraries on the user’s system instead of those provided by a base snap. Care must be taken to ensure that the application in the snap is linked against the correct libraries.

These guides show how classic confinement can be enabled for different types of projects, addressing any linking issues:

-  :ref:`Set up classic confinement for an autotools project <set-up-classic-confinement-for-an-autotools-project>`
-  :ref:`Set up classic confinement for a Python project <set-up-classic-confinement-for-a-python-project>`
-  :ref:`Set up classic confinement for a Makefile project <set-up-classic-confinement-for-a-makefile-project>`
-  :ref:`Set up classic confinement for a CMake project <set-up-classic-confinement-for-a-cmake-project>`
-  :ref:`Set up classic confinement for a Go project <set-up-classic-confinement-for-a-go-project>`

.. toctree::
   :hidden:

   set-up-classic-confinement-for-an-autotools-project
   set-up-classic-confinement-for-a-cmake-project
   set-up-classic-confinement-for-a-go-project
   set-up-classic-confinement-for-a-makefile-project
   set-up-classic-confinement-for-a-python-project
