.. 4233.md

.. _remote-reusable-parts:

Remote (reusable) parts
=======================

.. note::
          **PAGE DEPRECATED**

          Remote parts no longer work with recent versions of Snapcraft, and are not recommended.

          New versions of Snapcraft use Extensions to bundle a common set of parts and their dependencies. See :ref:`Snapcraft Extensions <snapcraft-extensions>` for further details.

Developers using snapcraft to build snaps can leverage pre-existing ‘remote parts’ in their snap, to save time re-implementing existing components.

Developers who have created local parts which may be of use to others, can submit them to the remote part cache for other snapcrafter’s to use.

Using remote parts
------------------

Search for parts
~~~~~~~~~~~~~~~~

The Snapcraft command line tool can search the remote parts cache using the ``search`` command. Before issuing ``search`` use the ``update`` to update the parts list from the remote cache.

::

   $ snapcraft update
   Downloading parts list

Once updated, ``search`` on its own will return the entire list of remote parts, or specify a search term to restrict the results returned.

::

   $ snapcraft search curl
   PART NAME  DESCRIPTION
   curl       A tool and a library (usable from many languages) for client side URL transfers, supporting FTP, FTPS, HTTP, HTTPS, TELNET, DICT, FILE and LDAP.

Viewing remote part contents
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The remote part definition can be viewed using the ``define`` command in snapcraft. This is useful for checking the contents of a remote part before incorporating it into your build.

::

   $ snapcraft define curl
   Maintainer: 'Sergio Schvezov <sergio.schvezov@ubuntu.com>'
   Description: A tool and a library (usable from many languages) for client side URL transfers, supporting FTP, FTPS, HTTP, HTTPS, TELNET, DICT, FILE and LDAP.

   curl:
     configflags:
     - --enable-static
     - --enable-shared
     - --disable-manual
     plugin: autotools
     snap:
     - -bin
     - -lib/*.a
     - -lib/pkgconfig
     - -lib/*.la
     - -include
     - -share
     source: http://curl.haxx.se/download/curl-7.44.0.tar.bz2
     source-type: tar

.. _using-remote-parts-1:

Using remote parts
~~~~~~~~~~~~~~~~~~

There are a few ways to use remote parts.

Implicit use
^^^^^^^^^^^^

The simplest use of a remote part is to specify it in the ``after`` stanza for an existing part. This will pull the remote part as the snap is built.

::

   parts:
       client:
          plugin: autotools
          source: .
          after: [curl]

Composing
^^^^^^^^^

Perhaps the remote part is almost what’s needed, but a change is required to fit the needs of your snap. In this case we can override pieces of the remote part. In this example we’re overriding the source URL.

::

   parts:
       client:
           plugin: autotools
           source: .
           after: [curl]
       curl:
           source: http://curl.haxx.se/download/curl-7.45.0.tar.bz2

Copy/Pasting
^^^^^^^^^^^^

In this example we take the output from ``snapcraft define <part>`` and paste it directly into our snap. This allows us full control over all the pieces of the part. We are now effectively no longer using the remote part, but have incorporated it into our build definition.

::

   parts:
       client:
           plugin: autotools
           source: .
           after: [curl]
       curl:
           configflags:
               - --enable-static
               - --enable-shared
               - --disable-manual
           plugin: autotools
           snap:
               - -bin
               - -lib/*.a
               - -lib/pkgconfig
               - -lib/*.la
               - -include
               - -share
           source: http://curl.haxx.se/download/curl-7.44.0.tar.bz2
           source-type: tar

Creating remote parts
---------------------

If you’ve created a part which might be useful for other developers, it’s possible to share them easily.

To create a remote part you create a snapcraft.yaml and the normal snap directory structure for your part.

The snap may NOT include any ‘apps’ or ‘hooks’ it MUST only contain ‘parts:’.

The remote part may actually contain one or more parts that you want to publish but all parts in the remote part MUST be published.

Publishing your remote part
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a repo (git, svn etc) containing only the part(s) you wish to share. The repo must be publicly visible!

For example https://github.com/sergiusens/curl contains the curl part mentioned above.

Update parts wiki
~~~~~~~~~~~~~~~~~

To make your remote part visible to the world (and discoverable by ‘snapcraft search’) you need to essentially catalog your remote part by adding it to the snapcraft parts page.

Add a yaml formatted entry to the `parts wiki <https://wiki.ubuntu.com/snapcraft/parts>`__ page.

For example the ``curl`` part is defined thus.

::

   ---
   origin: https://github.com/sergiusens/curl.git
   maintainer: Sergio Schvezov <sergio.schvezov@ubuntu.com>
   description:
     A tool and a library (usable from many languages) for
     client side URL transfers, supporting FTP, FTPS, HTTP,
     HTTPS, TELNET, DICT, FILE and LDAP.
   parts: [curl]
   ---

The ‘parts: [curl]’ is the critical piece as it MUST detail all of the parts contained in the snapcraft.yaml from your remote part.

*Note:* To edit the Ubuntu wiki you’ll need an `Ubuntu SSO <https://login.ubuntu.com/>`__ account (as used in the `snap store <https://dashboard.snapcraft.io/>`__), and need to request to join the `ubuntu-wiki-editors <https://launchpad.net/~ubuntu-wiki-editors>`__ team. Once approved, logout from the Ubuntu wiki and log back in again to refresh your new credentials.

Wait for cache refresh
~~~~~~~~~~~~~~~~~~~~~~

The online parts cache refreshes from the wiki every 30 minutes. You can check the status (including time of most recent update) of the parts cache at https://parts.snapcraft.io/v1/status.

If you have an error in your catalog entry on the parts wiki, the parts status page https://parts.snapcraft.io/v1/status will display details of the error. Your remote part won’t be published until you fix any errors.

Promote your new part
~~~~~~~~~~~~~~~~~~~~~

Consider starting a thread on the `forum <https://forum.snapcraft.io/>`__ to request feedback on, and promote the use of your new remote part.
