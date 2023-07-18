.. 25732.md

.. _snapcraft-metrics:

Snapcraft metrics
=================

The ``snapcraft metrics`` command is used to track installation and usage statistics for snaps published with your developer account.

A selection of metrics are also visible from the `Snap Store <https://snapcraft.io/store>`__ web UI. See `Snap Store metrics <https://snapcraft.io/docs/snap-store-metrics>`__ for details.

To ensure you have the correct and updated permissions to access *metrics*, we recommend using ``snapcraft logout`` and ``snapcraft login`` the first time this feature is used. See :ref:`snapcraft-metrics-credentials` for more details.


Usage
-----

The *snapcraft metrics* command takes the following arguments:

.. code:: bash

   snapcraft metrics <snap-name> --name <metric-name> \
   --start <start-date> --end <end-date> [--format=(json|table)]

+-----------------------+-------------------+-------------------------------+--------------------------------------------------------------------------------------------------------+
| Parameter             | Required/Optional | Type                          | Description                                                                                            |
+=======================+===================+===============================+========================================================================================================+
| ``<snap-name>``       | required          | string                        | Name of snap.                                                                                          |
+-----------------------+-------------------+-------------------------------+--------------------------------------------------------------------------------------------------------+
| ``name``              | required          | string                        | Name of supported metric (see below).                                                                  |
+-----------------------+-------------------+-------------------------------+--------------------------------------------------------------------------------------------------------+
| ``start``             | optional          | string of format *YYYY-MM-DD* | Start of date range to request (must not be later than today’s date). Defaults to yesterday.           |
+-----------------------+-------------------+-------------------------------+--------------------------------------------------------------------------------------------------------+
| ``end``               | optional          | string of format *YYYY-MM-DD* | End (inclusive) of date range to request (must not be later than today’s date). Defaults to yesterday. |
+-----------------------+-------------------+-------------------------------+--------------------------------------------------------------------------------------------------------+
| ``format``            | required          | string                        | Output format.                                                                                         |
+-----------------------+-------------------+-------------------------------+--------------------------------------------------------------------------------------------------------+


Supported metrics
~~~~~~~~~~~~~~~~~

The following metrics (``<metric-name>``) are supported:

* **daily_device_change**: contains the 3 series representing the number of **new**, **continued** and **lost** devices with the given snap installed compared to the previous day.
* **installed_base_by_channel**: contains one series per channel representing the number of devices with the given snap installed, channels with no data across the entire interval are omitted.
* **installed_base_by_country**: contains one series per country representing the number of devices with the given snap installed.
* **installed_base_by_operating_system**: contains one series per operating_system representing the number of devices with the given snap installed.
* **installed_base_by_version**: contains one series per version representing the number of devices with the given snap installed.
* **weekly_device_change**: similar to the ‘daily_device_change’ metric but operates on a 7 day window. i.e. **new** contains the number of devices that were seen during the last 7 days but not in the previous 7 day and so on for **continued** and **lost**.
* **weekly_installed_base_by_channel**: similar to the **installed_base_by_channel** metric but operates in a 7 day window.
* **weekly_installed_base_by_country**: similar to the **installed_base_by_country** metric but operates in a 7 day window.
* **weekly_installed_base_by_operating_system**: similar to the **installed_base_by_operating_system** metric but operates in a 7 day window.
* **weekly_installed_base_by_version**: similar to the **installed_base_by_version** metric but operates in a 7 day window.

Each metric has a query that includes a start and end date. The returned data, if available, will include all the days in-between. Weekly installed numbers still generated daily with the provided numbers being the averages for the 7 day window ending on the specified day.


Output format
-------------

The *snapcraft metrics* command outputs either as JSON string or a table.


JSON output
~~~~~~~~~~~

When using ``--format=json``, a JSON string matching the response from the Snap Store API server for the given metric is output.

This output is driven and provided by the Snap Store API and may be extended in the future.

Example **daily_device_change** output:

::

   $ snapcraft metrics my-snap --name daily_device_change \
   --start 2021-07-01 --end 2021-07-01 --format=json
   {'buckets': ['2021-07-01'], 'metric_name': 'daily_device_change',
   'series': [{'name': 'continued', 'values': [66]}, {'name': 'lost', 'values': [55]},
   {'name': 'new', 'values': [77]}], 'snap_id': '<snap-id>', 'status': 'OK'}

If the JSON object returned from the API is ``response``, *snapcraft* will output the pertinent data at ``response["metrics"][0]``. The relevant API documentation can be found on `dashboard.snapcraft.io <https://dashboard.snapcraft.io/docs/reference/v1/snap.html#the-metrics-response>`__.


Table output
~~~~~~~~~~~~

When using ``--format=table``, a table-based interpretation of the returned data, with columns for the requested date ranges and rows of the requested data series, is output.

As the output for most queries will be large, a pager is used to make the output more easily navigable. It is recommended that the user output this to a file and use an editor of choice.

   if the query returns with a “None” data point, it is replaced with a “-” to indicate zero (or not applicable depending on context).

Example **daily_device_change** output:

::

   $ snapcraft metrics my-snap --name daily_device_change \
   --start 2021-07-01 --end 2021-07-01 --format=table
   Devices    2021-07-01
   Continued  49
   Lost       21
   New        19

Example **installed_base_by_channel** output:

::

   $ snapcraft metrics my-snap --name installed_base_by_channel \
   --start 2021-07-01 --end 2021-07-01 --format=table
   Channel    2021-07-01  2021-07-02  2021-07-03
   Beta       245         255         240
   Candidate  1           1           0
   Edge       68          78          85
   Stable     401         405         409

Example **installed_base_by_country** output:

::

   $ snapcraft metrics my-snap --name installed_base_by_country \
   --start 2021-07-01 --end 2021-07-01 --format=table
   Country  2021-07-01  2021-07-02  2021-07-03
   Ar       6           6           6
   At       2           2           1
   Au       6           6           3
   Be       3           3           2
   Bg       1           2           1
   Br       14          14          10
   Ca       12          13          12
   Ch       3           3           2
   Cl       0           1           1
   Cn       3           2           2
   Co       1           1           1
   Cy       1           1           0
   Cz       1           1           0
   De       12          9           10
   Dk       1           2           1
   Es       9           10          8
   Fi       1           1           1
   Fr       8           8           8
   Gb       30          27          21
   Ge       0           1           1
   Gr       4           4           3
   Hk       1           1           1
   Hu       3           2           2
   Id       1           1           1
   Ie       2           2           2
   Im       3           3           3
   In       12          12          11
   It       6           9           4
   Jp       1           1           1
   Ke       1           1           1
   Lt       1           0           0
   Nl       4           4           6
   None     50          48          31
   Np       1           0           0
   Nz       2           3           3
   Pk       2           2           1
   Pl       5           5           4
   Pt       5           5           3
   Qa       1           1           1
   Ro       1           1           1
   Ru       5           5           5
   Se       8           7           6
   Sg       2           2           2
   Sk       4           2           2
   Tr       6           5           6
   Tw       5           5           5
   Us       53          44          39
   Uy       2           2           2
   Vn       1           0           0

Example **installed_base_by_operating_system** output:

::

   $ snapcraft metrics my-snap --name installed_base_by_operating_system --start 2021-07-01 --end 2021-07-01 --format=table
   OS                2021-07-01  2021-07-02  2021-07-03
   Arch/             2           1           1
   Centos/7          2           2           2
   Debian/10         3           3           2
   Elementary/5.1.7  2           2           2
   Elementary/6      1           0           0
   Fedora/34         1           1           1
   Linuxmint/20.1    1           1           1
   Manjaro/          1           2           1
   Pop/20.10         1           1           1
   Pop/21.04         1           1           1
   Ubuntu/16.04      10          10          11
   Ubuntu/18.04      68          66          77
   Ubuntu/19.04      1           1           0
   Ubuntu/19.10      1           1           1
   Ubuntu/20.04      255         260         250
   Ubuntu/20.10      9           9           7
   Ubuntu/21.04      88          92         99
   Ubuntu/21.10      2           2           5

Example **installed_base_by_version** output:

::

   $ snapcraft metrics my-snap --name installed_base_by_version --start 2021-07-01 --end
   Version  2021-07-01  2021-07-02  2021-07-03
   2.4.3             1           0           0
   2.4.4             1           1           1
   2.4.5             4           4           4
   2.5.0            28          28          16


.. _snapcraft-metrics-credentials:

Handling credentials
--------------------

The `Snap Store metrics API <https://dashboard.snapcraft.io/docs/reference/v1/snap.html#fetch-metrics-for-snaps>`__, used by the *snapcraft metrics* command, requires the ``package_metrics`` permission granted for the given credentials of the current snapcraft user.

Any currently logged in user will not have this permission granted to their existing cached credentials. They will likely require re-authentication to obtain it. Future logins will not require re-authentication as Snapcraft will request this permission during all future logins.

If you see an error such as the following, it’s likely you need to use ``snapcraft logout`` followed by ``snapcraft login`` to refresh your credentials:

.. code:: bash

   Errors:
   - Code: macaroon-permission-required
     Message: Permission "package_metrics" is required as a macaroon caveat.
     Extra: {'permission': 'package_metrics'}
