# Snapcraft Contribution Guide

Welcome to Snapcraft! We're a pretty friendly community and we're thrilled that
you want to make Snapcraft even better. However, we do ask that you follow some
general guidelines while doing so, just so we can keep things organized around
here.

**Prerequisite:** Sign the [contributor license agreement][1]. This is how you
give us permission to use your contributions.

1. Make sure a [Snapcraft bug][2] is filed for the bug you're about to fix, or
   feature you're about to add. This is not just paperwork required in order to
   land something: this is where you state what's happening, and why it's a
   shortcoming. Doing this before you start working on a fix also gives the
   Snapcraft team a chance to give you feedback and advice, which saves time
   for everyone!

2. We use a forking, feature-based workflow.

   Make a fork of Snapcraft, and create a branch named specifically for the
   feature on which you'd like to work. Make your changes there, adding new
   tests as needed, and make sure the existing tests continue to pass when your
   changes are complete (for information about running the tests, see the
   [HACKING][3] document).

3. Squash commits into one, well-formatted commit. If you really feel like there
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
       Make sure you mention the bug being fixed on a line all by itself at the
       end, like so:

       LP: #<bug number>
       ```

   Try to keep the summary to around 50 characters, and use the imperative mood.
   A good rule of thumb is that, if you extract the `<subsystem effected>` from
   the summary, it should be able to complete the following sentence:

       ```
       If applied, this commit will <insert summary here>.
       ```

4. Submit a pull request to get changes from your branch into master. Mention
   which bug is being resolved in the description of the pull request (bonus
   points if it's a hyperlink to the bug itself).

[1]: http://www.ubuntu.com/legal/contributors/
[2]: https://bugs.launchpad.net/snapcraft
[3]: HACKING.md
