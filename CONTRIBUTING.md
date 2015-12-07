# Snapcraft Contribution Guide

Welcome to Snapcraft! We're a pretty friendly community and we're thrilled that
you want to make Snapcraft even better. However, we do ask that you follow some
general guidelines while doing so, just so we can keep things organized around
here.

1. Sign the [contributor license agreement][1].

   This is how you give us permission to use your contributions.

2. We use a forking, feature-based workflow.

   Make a fork of Snapcraft, and create a branch named specifically for the
   feature on which you'd like to work. Make your changes there, adding new
   tests as needed, and make sure the existing tests continue to pass when your
   changes are complete (for information about running the tests, see the
   [HACKING][2] document).

3. Squash commits into one, well-formatted commit.

   If you really feel like there should be more than one commit in your branch,
   then you're probably trying to introduce more than one feature and you should
   make another branch for it.

4. Submit a pull request to get changes from your branch into master.

   If you want to get the change into 1.x as well, make a note of it on the pull
   request and it can be cherry-picked after the merge.

[1]: http://www.ubuntu.com/legal/contributors/
[2]: HACKING.md
