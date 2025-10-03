.. |app| replace:: Snapcraft
.. |artifact| replace:: snap
.. |an-artifact| replace:: a snap
.. |app-command| replace:: snapcraft

.. _how-to-reuse-packages-between-builds:

.. include:: ../../common/craft-application/how-to-guides/reuse-packages-between-builds.rst
    :end-before: .. Uncomment and customise this block for your app


Integrate with |app|
--------------------

With the container for the proxy server configured and running in the background, you
can begin accessing the APT cache with |app|.

When you launch |app|, pass the proxy server to it with the ``http_proxy``
environment variable.

.. code-block:: bash
    :substitutions:

    http_proxy="http://package-cache.lxd:3128/" |app-command| pack

Alternatively, you can pass the IP address of the proxy server. Copy the IP address from
LXC:

.. code-block:: bash
    :substitutions:

    lxc list --project |app-command| | grep package-cache

Then, when launching |app|, pass the IP address to ``http_proxy``:

.. code-block:: bash
    :substitutions:

    http_proxy="http://<package-cache-ip>:3128/" |app-command| pack


.. include:: ../../common/craft-application/how-to-guides/reuse-packages-between-builds.rst
    :start-after: http_proxy="http://<package-cache-ip>:3128/" |app-command| pack
