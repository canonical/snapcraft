# Snapcraft Contribution Guide

Welcome to Snapcraft! We're a pretty friendly community and we're thrilled that
you want to make Snapcraft even better. However, we do ask that you follow some
general guidelines while doing so, just so we can keep things organized around
here.

**Prerequisite:** Sign the [contributor license agreement][1]. This is how you
give us permission to use your contributions.

1. If there is a [Snapcraft bug][2] you are trying to fix, please refer to
   it here. If it is a feature that has not been discussed, please raise
   awareness on https://forum.snapcraft.io under the *snapcraft* topic. This
   will ensure that we're all on the same page, and your work is not in vain
   or duplicating what someone else is already doing. This actually saves time!

2. We use a forking, feature-based workflow.

   Make a fork of Snapcraft, and create a branch named specifically for the
   feature on which you'd like to work. Make your changes there, adding new
   tests as needed, and make sure the existing tests continue to pass when your
   changes are complete (for information see the [HACKING][3] and [TESTING][4]
   documents).

3. We try to follow a consistent and readable code style. Read the
   [CODE_STYLE][5] document and please make sure that your code complies.

4. Squash commits into one, well-formatted commit. If you really feel like there
   should be more than one commit in your branch, then you're probably trying to
   introduce more than one feature and you should make another branch for
   it.

   This is important: your commit diff says what changed, but only the commit
   message can say why the change was necessary. In an effort to take good care
   of our `git log`, we try to follow this template for commit messages:


       ```
       <subsystem effected>: lower-case summary of changes

       More detailed explanatory text, if necessary. Wrap it to 72 characters.
       Think of this like an email, where you have a subject line and a body.

       ```

   Try to keep the summary to around 50 characters, and use the imperative mood.
   A good rule of thumb is that, if you extract the `<subsystem effected>` from
   the summary, it should be able to complete the following sentence:

       ```
       If applied, this commit will <insert summary here>.
       ```

5. Submit a pull request to get changes from your branch into master. Mention
   which bug is being resolved in the description of the pull request (bonus
   points if it's a hyperlink to the bug itself).

[1]: http://www.ubuntu.com/legal/contributors/
[2]: https://bugs.launchpad.net/snapcraft
[3]: HACKING.md
[4]: TESTING.md
[5]: CODE_STYLE.md