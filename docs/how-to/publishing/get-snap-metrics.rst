.. _how-to-get-snap-metrics:

Get snap metrics
================

Snap stores collect installation statistics for snaps you authored, and you can retrieve
the collected data with Snapcraft.

For a complete list of available snap metrics, see :ref:`reference-metrics`.


Prerequisites
-------------

As snap metrics are confidential, only a snap's author can access them.

Before you begin, log in to your Snapcraft account.

.. link to :ref:`log-in-to-a-store`


Get all data for a metric
-------------------------

The most basic query for a snap metric, which returns all historical data for it, is:

.. code-block:: bash

    snapcraft metrics <snap-name> --name <metric-name> --format=table

Replace ``<metric-name>`` with the statistic you want.

The returned data, if available, consists of the daily value for the metric across snap
versions:

.. terminal::

    Version  2025-03-03  2025-03-04  2025-03-05  2025-03-06  2025-03-07  2025-03-08
    V0.27.0  21          20          22          16          0           0
    V0.28.2  0           0           0           3           19          20

As the terminal output for most queries will be large, the command uses a pager so you
can more easily navigate the results. It's even easier if you write the output to a
file and then review it in a text editor.

If no data is available, the output is empty.


Get data within a date range
----------------------------

You can confine the time span of the metric by providing one or both of the ``--start``
and ``--end`` arguments:

.. code-block:: bash

    snapcraft metrics <snap-name> --name <metric-name> --format=table \
      --start <yyyy-mm-dd> --end <yyyy-mm-dd>

The length of the date range can't exceed five years, and the end date can't be in the
future.


Format metric data as JSON
--------------------------

If you need to format the return data as a JSON object, change the value of the
``--format`` argument:

.. code-block:: bash

    snapcraft metrics <snap-name> --name <metric-name> --format=json

The returned data then takes the form of a JSON object:

.. dropdown:: JSON object for a metric

    .. code-block:: json

      {
        "buckets": [
          "2025-03-03",
          "2025-03-04",
          "2025-03-05",
          "2025-03-06",
          "2025-03-07",
          "2025-03-08"
        ],
        "metric_name": "installed_base_by_version",
        "series": [
          {
            "name": "v0.27.0",
            "values": [
              21,
              20,
              22,
              16,
              null,
              null
            ]
          },
          {
            "name": "v0.28.2",
            "values": [
              null,
              null,
              null,
              3,
              19,
              20
            ]
          }
        ],
        "snap_id": "q9HYHk05OMrPAvzSb1q6AXkmZR6rkDgx",
        "status": "OK"
      }


Troubleshoot access to metric data
----------------------------------

The `Snap Store metrics API
<https://dashboard.snapcraft.io/docs/reference/v1/snap.html#fetch-metrics-for-snaps>`_,
called by the ``snapcraft metrics`` command, requires your account to have the
package_metrics permission.

If this is your first time querying a snap metric after you've registered your Snapcraft
account, start by logging in and out of Snapcraft to refresh your session permissions.
Otherwise, you might get this error:

.. terminal::

    Errors:
    - Code: macaroon-permission-required
      Message: Permission "package_metrics" is required as a macaroon caveat.
      Extra: {'permission': 'package_metrics'}

After you successfully access a metric, future queries won't require re-authentication.
