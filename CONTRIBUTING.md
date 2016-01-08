# Snapcraft Contribution Guide

Welcome to Snapcraft! We're a pretty friendly community and we're thrilled that
you want to make Snapcraft even better. However, we do ask that you follow some
general guidelines while doing so, just so we can keep things organized around
here.

**Prerequisite:** Sign the [contributor license agreement][1]. This is how you
give us permission to use your contributions.

1. Make sure a [Snapcraft bug][2] is filed for the bug you're about to fix, or
   feature you're about to add. If it's a feature, mark the status as
   "Wishlist."

2. We use a forking, feature-based workflow.

   Make a fork of Snapcraft, and create a branch named specifically for the
   feature on which you'd like to work. Make your changes there, adding new
   tests as needed, and make sure the existing tests continue to pass when your
   changes are complete (for information about running the tests, see the
   [HACKING][3] document).

3. Squash commits into one, well-formatted commit. Mention the bug being
   resolved in the commit message on a line all by itself like `LP: #<bug>`.

   If you really feel like there should be more than one commit in your branch,
   then you're probably trying to introduce more than one feature and you should
   make another branch for it.

4. Submit a pull request to get changes from your branch into master. Mention
   which bug is being resolved.

   If you want to get the change into 1.x as well, make a note of it on the pull
   request and it can be cherry-picked after the merge.

[1]: http://www.ubuntu.com/legal/contributors/
[2]: https://bugs.launchpad.net/snapcraft
[3]: HACKING.md
