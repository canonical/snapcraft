.. _reference-channels:

Channels
========

Snaps are published on *channels*, which are concurrent versions of the snap,
addressing needs such as preview releases and long-term version support.

A channel is composed of three components -- a track, a risk, and a branch.

Briefly, a channel is represented in Snapcraft like this:

  <track>/<risk>/<branch>

In Snapcraft commands, a typical channel without a branch looks like this:

.. code:: bash

  --channel=latest/edge


.. _reference-channels-track:

Track
-----

The *track* represents the highest level of organization for the snap's
release. Typically, it signifies a major release. Authors can use tracks to
maintain different major versions of their software.

All snaps have one default track, called *latest*. Most operations and commands
within Snapcraft and the snap ecosystem assume this track, unless otherwise
specified. If a snap has need for other tracks, they must be vetted and granted
on a snap-by-snap basis by the Snap Store team. The process for requesting a
track is outlined in `Process for aliases, auto-connections and tracks
<https://snapcraft.io/docs/process-for-aliases-auto-connections-and-tracks>`_.

The release semantics for a track is at the snap author's discretion. A track
could, for examples, encompass minor updates, major updates, or long-term
support releases. Or, it could signify something else entirely.


.. _reference-channels-risk:

Risk
----

The *risk* represents the author's confidence in the release, its readiness,
and signals its potential dangers to users. This is the most important
component of a channel. With it, authors can maintain multiple variations of
the same release of the snap under the same name.

Risk levels are preset by the Snap Store. The available levels are *stable*,
*candidate*, *beta*, and *edge*. Snaps built with the devel grade can't be
released to either the stable or candidate channels.

Snaps are installed to a user's system with the stable risk level by default.


.. _reference-channels-branch:

Branch
------

The *branch* represents the smallest meaningful unit of history in the project's
development. It's an optional component intended to distinguish short-lived or on-demand
releases for bug-fixing, one-off testing, and other irregular work.

Good branch names convey their purpose, such as ``fix-bug123``. Branch names are not,
however, discoverable in the Snap Store. Instead, they must be shared by authors to
users and referenced by their exact name.

If a branch has no new revisions for 30 days, the Snap Store closes the branch. The
replacement snap will then be chosen from the next most conservative risk-level from the
same track. For example, after the ``beta/fix-bug123`` branch closes, its name will
redirect to the latest revision in ``beta``.

Branch revisions can also be promoted or demoted to other channels, such as a
``latest/edge/bugfix-123`` branch being promoted to the snap's ``latest/stable``
channel. When this happens, the revision's lifecycle will match the channel schema. If
promoted to a stable channel, it would be a long-lived revision. If promoted to a new
branch, the new branch would start its own 30-day lifecycle.
