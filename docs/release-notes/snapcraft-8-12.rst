.. _release-8.12:

Snapcraft 8.12 release notes
============================

.. add date here, once scheduled

Learn about the new features, changes, and fixes introduced in Snapcraft 8.12.


Requirements and compatibility
------------------------------
See :ref:`reference-system-requirements` for information on the minimum hardware and
installed software.


What's new
----------

Snapcraft 8.12 brings the following features, integrations, and improvements.


Minor features
--------------

Remote builds: ``--build-for`` and shorthand platforms
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Previously, the :ref:`remote builder <explanation-remote-build>` couldn't accept the
``--build-for`` argument when building core24 snaps with a shorthand :ref:`platforms
<reference_architectures>` definition.

Due to improvements made by the Launchpad team, ``--build-for`` can now be used when
building core24 snaps remotely.


Improved usability when piping ``status`` output
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The output of ``snapcraft status`` was reworked to allow for easier parsing.
Prior to the change, the output was formatted as follows:

.. terminal::
    :input: snapcraft status snapcraft | grep pr- | head
    
                      edge/pr-5718  8.11.1.post15             15744       -           2025-09-19T00:00:00Z
                      edge/pr-5715  8.11.1.post10             15719       -           2025-09-18T00:00:00Z
                      edge/pr-5714  8.11.1.post10             15712       -           2025-09-18T00:00:00Z
                      edge/pr-5710  8.11.1.post5              15703       -           2025-09-17T00:00:00Z
                      edge/pr-5706  8.11.1.post10             15702       -           2025-09-15T00:00:00Z
                      edge/pr-5705  8.11.1.post11             15721       -           2025-09-18T00:00:00Z
                      edge/pr-5703  8.11.1.post6              15673       -           2025-09-13T00:00:00Z
                      edge/pr-5701  8.11.1.post7              15686       -           2025-09-14T00:00:00Z
                      edge/pr-5700  8.11.1.post7              15681       -           2025-09-13T00:00:00Z
                      edge/pr-5699  8.11.1.post4              15666       -           2025-09-13T00:00:00Z

The same status query now produces the following output:

.. terminal::
    :input: snapcraft status snapcraft | grep pr- | head

    latest   amd64    edge/pr-5718  8.11.1.post15             15744       -           2025-09-19T00:00:00Z
    latest   amd64    edge/pr-5715  8.11.1.post10             15719       -           2025-09-18T00:00:00Z
    latest   amd64    edge/pr-5714  8.11.1.post10             15712       -           2025-09-18T00:00:00Z
    latest   amd64    edge/pr-5710  8.11.1.post5              15703       -           2025-09-17T00:00:00Z
    latest   amd64    edge/pr-5706  8.11.1.post10             15702       -           2025-09-15T00:00:00Z
    latest   amd64    edge/pr-5705  8.11.1.post11             15721       -           2025-09-18T00:00:00Z
    latest   amd64    edge/pr-5703  8.11.1.post6              15673       -           2025-09-13T00:00:00Z
    latest   amd64    edge/pr-5701  8.11.1.post7              15686       -           2025-09-14T00:00:00Z
    latest   amd64    edge/pr-5700  8.11.1.post7              15681       -           2025-09-13T00:00:00Z
    latest   amd64    edge/pr-5699  8.11.1.post4              15666       -           2025-09-13T00:00:00Z
