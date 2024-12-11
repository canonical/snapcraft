.. _channels:

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


Track
------

The *track* represents the highest level of organisation for the snap's
release. Typically, it signifies a major release. Authors can use tracks to
maintain different major versions of their software.

All snaps must have a default track called *latest*. This track is assumed for
most operations with Snapcraft, unless otherwise specified. Aside from latest,
the release convention for tracks is at the author's discretion. A track could,
as examples, be used to track minor updates (2.0.1, 2.0.2), major updates (2.1,
2.2), or releases held for long-term support (3.2, 4.1).


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


Branch
------

The *branch* represents the smallest meaningful unit of history in the
project's development. It's an optional component intended to distinguish
short-lived or on-demand releases for bug-fixing, one-off testing, and other irregular work.

Branch names convey their purpose, such as ``fix-for-bug123``. Branch names are
not, however, discoverable in the Snap Store. Instead, they must be shared by
authors to users and referenced by their exact name. If a snap branch has no
new publications for 30 days, the Snap Store closes its branch.
