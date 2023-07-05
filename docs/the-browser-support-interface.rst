.. 7775.md

.. _the-browser-support-interface:

The browser-support interface
=============================

``browser-support`` allows access to various APIs needed by modern web browsers.

**Auto-connect**: no when ``allow-sandbox: true``, yes otherwise **Attributes**:

* ``allow-sandbox: true|false`` (defaults to ``false``)

This interface is intended to be used when using an embedded Chromium Content API or using the sandboxes in major browsers from vendors like Google and Mozilla. The ``allow-sandbox`` attribute may be used to give the necessary access to use the browser’s sandbox functionality.

   ⓘ A browser’s internal sandbox requires numerous privileged security policy rules to work and is typically considered trusted outside of *snapd*. For this reason, ``--allow-sandbox=true`` is limited to trusted publishers only.

For some web applications, such those using Electron, it is often useful to disable the internal sandbox and rely on strict confinement, forcing the snap to use ``allow-sandbox: false``. To do this, specify ``--no-sandbox`` on the command line for your application.

When ``--no-sandbox`` is used, the snap is confined regardless of the ``--allow-sandbox`` attribute, which now only controls whether or not the browser’s internal sandbox can be used.

   ⓘ This is a snap interface. See :ref:`Interface management <interface-management>` and :ref:`Supported interfaces <supported-interfaces>` for further details on how interfaces are used.
