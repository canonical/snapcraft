.. _release-notes:

Release notes
=============

This page lists the notes for past releases of Starcraft, which summarise new features,
bug fixes and backwards-incompatible changes in each version. It also contains the
release and support policies for Starcraft.


Current releases
----------------


<latest release>
~~~~~~~~~~~~~~~~

- <link to latest release, update, such as 8.2.0>
- <link to latest release, update, such as 8.1.0>
- <link to latest release, initial, such as 8.0.0>


<parallel release>
~~~~~~~~~~~~~~~~~~

<If necessary, add guidance and caveats about these older releases, such as
"Snapcraft 7 is available for building core18 snaps. When building for a newer
base, use Snapcraft 8.">

- <link to parallel release, update, such as 7.2.0>
- <link to parallel release, update, such as 7.1.0>
- <link to parallel release, initial, such as 7.0.0>


Past releases
-------------


<past release>
~~~~~~~~~~~~~~

- <link to past release, update, such as 6.2.0>
- <link to past release, update, such as 6.1.0>
- <link to past release, initial, such as 6.0.0>


.. _release-versioning:

Release versioning
------------------

Starcraft version naming follows the Semantic Versioning 2.0.0 scheme with
numbers for major, minor, and patch versions.

.. list-table::
    :header-rows: 1

    * - Version
      - Example
      - Significance
    * - Major
      - **3**.1.2
      - <Apps: "A change that drops support for an earlier software base.">

        <Libraries: "A change that breaks compatibility with the previous
        version.">
    * - Minor
      - 3.\ **1**\ .2
      - A new feature within the major version.
    * - Patch
      - 3.1.\ **2**
      - A bug fix within the major or minor version.


Long-term support
-----------------

Starcraft doesn't have long-term support (LTS) releases. However, we typically
deliver a compatibility release shortly after Ubuntu LTS releases to ensure
continuity.

<Apps: Starcraft software bases are derived from Ubuntu LTS releases, and their
development keeps pace with the OS's new releases and support lifecycle.>

.. toctree::
   :maxdepth: 1


.. release note template:

  Starcraft 2.0 release notes
  ===========================

  15 October 2024

  Learn about the new features, changes, and fixes introduced in Starcraft 2.0.


  Requirements and compatibility
  ------------------------------

  Starcraft 2.0 requires Python 3.11 or higher.

  <If there are multiple requirements, remove "Python 3.11 or higher" in the
  previous paragraph and add a separate list here, with the same format of
  "<package> or higher".>

  For development and testing, Starcraft requires a <architecture> system or VM
  with a minimum of <number>GB RAM.


  What's new
  ----------

  Starcraft 2.0 brings the following features, integrations, and improvements.


  <Important change>
  ~~~~~~~~~~~~~~~~~~

  <Try and stay within these guidelines when writing headings for important
  changes.>

  +----------------------------------------+------------------------------+-----------------------------------------+
  | Type of change                         | Heading format               | Example                                 |
  +========================================+==============================+=========================================+
  | New feature                            | <Name of feature>            | Snap deltas                             |
  +----------------------------------------+------------------------------+-----------------------------------------+
  | Support for technology, or integration | Support for <technology>     | Support for signed commits              |
  |                                        | <Technology> integration     | Gnome integration                       |
  +----------------------------------------+------------------------------+-----------------------------------------+
  | Improvement to existing feature        | <Describe improvement>       | Faster buildset queries                 |
  |                                        | Improved <aspect of feature> | Improved language of buildset queries   |
  +----------------------------------------+------------------------------+-----------------------------------------+
  | Other important update                 | <Describe update>            | Mitigation for Heartbleed vulnerability |
  +----------------------------------------+------------------------------+-----------------------------------------+

  <Paragraph 1, optional: Briefly cover the previous behaviour or the change in
  circumstances. For example, "With Ubuntu 24.04 LTS, the Snap Store and App
  Center now collect public reviews for snaps and assign an averaged score to
  them to provide users and authors an avenue for discoverability and
  feedback.">

  <Paragraph 2: Present the new behaviour or feature. In words, *show* what the feature
  is and make a case for how the reader could benefit from it. Centre the user whenever
  possible ("you"), and speak on behalf of Canonical ("we"). Prefer general, simple
  usage over complex applications. Use past tense, or the form "is now [x]" or "now
  [does x]". For example, "We understand that some authors may not want to have their
  snaps publicly ranked. If you prefer to disable ranking for your snap, we added the
  ``feedback`` key in Snapcraft recipes, which contains child keys for controlling many
  of the rating and feedback features in the store. You can declare ``voting: false`` to
  disable voting." Another example: "The Maven and Ant plugins now generate the more
  standard path to the Java runtime executable instead of an unconventional one, making
  their locations more predictable.">

  <Paragraph 3, optional: Provide a call to action. This could take several
  forms, such as a call to immediately perform a relevant action in Starcraft,
  solicititation of the reader's feedback on a form or forum, or a link to
  documentation, demo, blog post, and so on. For example, "See ``:ref:`Manage
  store profile``` to configure how the public can engage with your snap on the
  store".>


  Minor features
  --------------

  Starcraft 2.0 brings the following minor changes.


  <Feature A>
  ~~~~~~~~~~~

  <Add a short list of changes to the feature. Keep each item brief and for the most
  part descriptive. There's little need to sell the change or give a detailed reason.
  Use past tense, or the form "is now [x]" or "now [does x]". For example, "- Made the
  error message for ``method()`` more descriptive and recommend a likely remedy."
  Another example: "- The GET method on the profiles API now returns the user creation
  date.">


  Backwards-incompatible changes
  ------------------------------

  The following changes are incompatible with previous versions of Starcraft.


  <"Removed" or "Disabled"> <feature B>
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  <Paragraph 1, optional: Briefly cover the sequence of events that led to this
  feature's removal. Use past tense. For example, "SP ABC-123 by the NIST,
  published in October 2024, showed that algorithm X is no longer considered
  adequate for protecting data in transit".>

  <Paragraph 2: State precisely what was removed or disabled. Advise on an
  alternative solution, or state if no alternative exists. If necessary,
  describe the consequences of the reader's inaction. Link to relevant
  documentation, standards, or public discussion. For example, "In accordance
  with the report, Starcraft 2.0 no longer supports encryption algorithm X. As
  of this release, if you haven't already we highly recommend you immediately
  switch to encryption algorithm Y to ensure your data stays protected. For
  more details about this decision and our policy, see ```Security notice on
  encryption X <>`_`` on the Ubuntu blog.">


  Feature deprecations
  --------------------

  The following features are deprecated in Starcraft 2.0:

  Deprecated <feature C>
  ~~~~~~~~~~~~~~~~~~~~~~

  <Use the same format as backwards-incompatible changes, but use present tense
  to describe what is deprecated in this release, and how. Advise on an
  alternative replacement, if it exists. This item is a statement of fact, not
  a promise. If the implementation or schedule for the deprecation changed from
  what we originally planned, don't make a point of it -- simply describe what
  *is*. End by linking to relevant documentation, standards, or public
  discussion. For example, "In October 2024, the NIST published SP ABC-123,
  urging software publishers to cease the use of encryption algorithm X. We are
  deprecating its usage in this release, and advise you to adopt algorithm Z as
  a replacement. For more details about this decision and our policy, see
  `Security notice on encryption X <>`_ on the Ubuntu blog.">


  Scheduled feature deprecations
  ------------------------------

  <Iterate on the following paragraph+items if you're covering multiple
  versions in this section.>

  The following features will be deprecated in Starcraft <planned version>:


  <Feature D>
  ~~~~~~~~~~~

  <Future deprecation: Use the same format as backwards-incompatible changes,
  but use future tense to describe what we *intend* and *plan* to do in
  subsequent releases. Think of this as a promise or commitment to the reader,
  and be mindful of the trust we want them to place in us. Don't write
  conjecture or make promises about details that haven't been decided. Include
  only the decisions that we have set in stone and information we're allowed to
  disclose at of the release day. Use phrases like "we plan to", "we are
  working on", or "we have scheduled development of". End by linking to
  relevant documentation, standards, or public discussion. For example, "In
  October 2024, the NIST published SP ABC-123, urging software publishers to
  cease the use of encryption algorithm X. We plan to deprecate it in Starcraft
  1.2. For more details about this decision and our policy, see `Security
  notice on encryption X <>`_ on the Ubuntu blog.">


  Known issues
  ------------

  The following issues were reported and are scheduled to be fixed in upcoming
  patch releases.

  See individual issue links for any mitigations.

  - `ID <link>`_ <Title>
  - `ID <link>`_ <Title>


  Fixed bugs and issues
  ---------------------

  The following issues have been resolved in Starcraft 2.0:

  - `ID <link>`_ <Title>
  - `ID <link>`_ <Title>


  Contributors
  ------------

  We would like to express a big thank you to all the people who contributed to
  this release.

  :literalref:`@alex<https://example.com/alex>`,
  :literalref:`@blair<https://example.com/blair>`,
  :literalref:`@cam<https://example.com/cam>`,
  and :literalref:`@devin<https://example.com/devin>`
