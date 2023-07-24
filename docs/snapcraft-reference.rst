.. 31051.md

.. _snapcraft-reference:

Reference
=========

.. note::
          **NOTE TO EDITORS**

          This topic is currently under construction as we start to migrate our current documentation to use the `Diátaxis <https://diataxis.fr/>`__ framework.

          See `Diátaxis, a new foundation for Canonical documentation <https://ubuntu.com/blog/diataxis-a-new-foundation-for-canonical-documentation>`__ for more details.



Our Snapcraft *Reference section* is for when you need to know which plugins we offer, which interfaces you can use, and what you can add to snapcraft.yaml.

+------------------------------------------------------------------+--------------------------------------------------------------------+
| **Reference**                                                    | How Snapcraft works                                                |
+==================================================================+====================================================================+
| :ref:`Snapcraft.yaml reference <snapcraft-yaml-reference>`       | Every :file:`snapcraft.yaml` keyword listed, described and defined |
+------------------------------------------------------------------+--------------------------------------------------------------------+
| :ref:`Supported plugins <supported-plugins>`                     | All currently supported language, platform, and toolkit plugins    |
+-------------------------------------------------------------------+-------------------------------------------------------------------+
| :ref:`Parts environment variables <parts-environment-variables>` | Variables that can be used within a part’s build environment       |
+-------------------------------------------------------------------+-------------------------------------------------------------------+

Alternatively, the Snapcraft tutorials section contain step-by-step tutorials to help outline what Snapcraft is capable of while helping you achieve specific aims, such as installing Snapcraft and building your first snap.

If you have a specific goal, but are already familiar with Snapcraft, our How-to guides have more in-depth detail than our tutorials and can be applied to a broader set of applications. They’ll help you achieve an end-result but may require you to understand and adapt the steps to fit your specific requirements.

Finally, for a better understanding of how Snapcraft works, and how it can be used and configured, our *Explanation* section enable you to expand your knowledge and become better at building snaps with Snapcraft.

.. toctree::
   :hidden:

   architectures
   base-snaps
   reference/commands
   documentation-guidelines
   environment-variables
   environment-variables-that-snapcraft-exposes
   glossary
   supported-plugins
   supported-extensions
   supported-interfaces
   supported-snap-hooks
   parts-environment-variables
   snapcraft-package-repositories
   build-options
   build-providers
   revisions
   snapcraft-advanced-grammar
   snapcraft-and-extended-security-maintenance
   snapcraft-app-and-service-metadata
   snapcraft-authentication-options
   snapcraft-linters
   snapcraft-parts-metadata
   snapcraft-plugin-api
   snapcraft-release-notes
   snapcraft-top-level-metadata
   snapcraft-yaml-reference
   super-privileged-interfaces
   the-kernel-snap
   the-snap-format
   the-snap-refresh-control-interface
   the-snap-themes-control-interface
   the-snapcraft-yaml-schema
   the-snapd-control-interface
   validation-sets
