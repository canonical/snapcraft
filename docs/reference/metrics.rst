.. _reference-metrics:

Metrics
=======

This page lists all supported statistics that stores gather about snaps.

All metrics are generated daily. Those with seven-day windows provide daily figures, but
these are not raw values for each day. Rather, they are the weekly average.

.. list-table::
    :header-rows: 1
    :widths: 1 2

    * - Metric
      - Data
    * - ``daily_device_change``
      - Contains the three series representing the number of new, continued and lost devices with the given snap installed compared to the previous day.
    * - ``installed_base_by_channel``
      - Contains one series per channel representing the number of devices with the
        given snap installed. Channels with no data across the entire interval are
        omitted.
    * - ``installed_base_by_country``
      - Contains one series per country representing the number of devices with the
        given snap installed.
    * - ``installed_base_by_operating_system``
      - Contains one series per operating system representing the number of devices with
        the given snap installed.
    * - ``installed_base_by_version``
      - Contains one series per version representing the number of devices with the
        given snap version installed.
    * - ``installed_base_by_architecture``
      - Contains one series per architecture representing the number of devices with the
        given snap installed.
    * - ``weekly_device_change``
      - Similar to the ``daily_device_change`` metric but operates on a seven-day
        window. In other words, contains the number of devices that were seen during the
        last seven days but not in the previous seven day and so on for continued and lost.
    * - ``weekly_installed_base_by_channel``
      - Similar to the ``installed_base_by_channel`` metric but operates in a seven-day
        window.
    * - ``weekly_installed_base_by_country``
      - Similar to the ``installed_base_by_country`` metric but operates in a seven-day
        window.
    * - ``weekly_installed_base_by_operating_system``
      - Similar to the ``installed_base_by_operating_system`` metric but operates in a
        seven-day window.
    * - ``weekly_installed_base_by_version``
      - Similar to the ``installed_base_by_version`` metric but operates in a seven-day
        window.
    * - ``weekly_installed_base_by_architecture``
      - Similar to the ``installed_base_by_architecture`` metric but operates in a
        seven-day window.
