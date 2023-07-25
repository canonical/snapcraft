.. 11486.md

.. _adding-global-metadata:

Adding global metadata
======================

Global metadata attributes are used within :ref:`snapcraft.yaml <the-snapcraft-yaml-schema>` to identify the snap locally, and after publishing to the `Snap Store <https://snapcraft.io/store>`__, to identify your snap to users and potential users.

Attributes include a snap’s name and description, its level of confinement, and where the application icon can be found.

.. code:: yaml

   name: qfsview
   summary: Visualise storage utilisation.
   description: |
       qFSView displays files and folders as a rectangle with an
       area proportional to the storage they and their children use.
   version: 1.0
   icon: gui/qfsview.png
   base: core18
   grade: stable
   confinement: strict

For the complete list of global metadata, see :ref:`Snapcraft top-level metadata <snapcraft-top-level-metadata>`.

Recommended global metadata
---------------------------

Global metadata is a mixture of mandatory and optional values.

You can generate a buildable template of both required and recommended values with the `snapcraft init` command in a new project folder (see :ref:`Snapcraft overview <snapcraft-overview>` for more details).

The following attributes are mandatory:

-  **name** A snap’s name is important. It must start with an ASCII character and can only contain 1) letters in lower case, 2) numbers, and 3) hyphens, and it can’t start or end with a hyphen. For the `Snap Store <https://snapcraft.io/store>`__, it also needs to be both unique and easy to find.

   For help on choosing a name and registering it on the Snap Store, see :ref:`Registering your app name <registering-your-app-name>`.

-  **summary** The *summary* is a short descriptive sentence to tell prospective users about an application’s primary purpose, in fewer than 80 characters.

-  **description** Unlike the *summary*, the description can be as verbose as you need it to be. The above snippet shows the description text following a pipe symbol (``|``), which is used in YAML to maintain newline formatting in multiline text blocks.

   The following, for example, will ensure both *Line one* and *Line two* appear on separate lines:

   .. code:: yaml

      description: |
          Line one
          Line two

   While you shouldn’t write thousands of words, the more details you provide, the more likely people are to discover and use your application. Feature lists, update descriptions, a brief *Getting started* guide, are legitimate uses for the *description*.

-  **version** While having a value for *version* is mandatory, its value can be anything. Setting this to something like ``test`` makes sense while you’re first building your snap, and you can later replace this with a specific version, or a reference to a script that replaces the version number automatically.

   The value for *version* is also commonly imported for external metadata. See :ref:`Using using external metadata <using-external-metadata>` for further details.

The following attributes should also be included:

-  **base** A base snap is a special kind of snap that provides a run-time environment with a minimal set of libraries that are common to most applications.

   See :ref:`Base snaps <base-snaps>` for help selecting a base for your snap.

-  **grade** This should initially be ``devel`` and changed to ``stable`` when you have a snap ready for release.

-  **confinement** A snap’s confinement level is the degree of isolation it has from your system. When first building a snap, set this to ``devmode`` to initially limit the side-effects of confinement until you have a working snap.

   See :ref:`Snap confinement <snap-confinement>` for further details.

For convenience, and to help avoid duplicating sources, external metadata such as :ref:`AppStream <meta-appstream>` can be imported into :file:`snapcraft.yaml`. See :ref:`Using external metadata <using-external-metadata>` for further details.

Two further global attribites are ``apps:`` and ``parts:``. These expand into separate sections that deal with how your snap is built and where its various resources are located. See :ref:`Adding parts <adding-parts>` for the next logical step in snap building.
