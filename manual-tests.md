# Test log in with one-time password

1. Set up an SSO account with two-factor authentication.
2. Run snapcraft logout
3. Run snapcraft login
4. Enter the email address.
5. Enter the password.
6. Enter the one-time password.

   * Check that the log in was successful.


# Test file ownership is retained

1. 'snapcraft build' a simple snap
2. sudo touch install/test-owner-file
3. sudo chown nobody:nogroup install/test-owner-file
4. sudo snapcraft prime
5. ensure that prime/test-owner-file is owned by nobody and nogroup

# Test stage package caching

1. `snapcraft pull` a snap that has `parts` with `stage-packages`.
2. Run `snapcraft clean`.
3. Verify there is cached apt data in `~/.cache/snapcraft/<hash>/`
4. Run `snapcraft pull` again and notice the download is minimal.
5. Wipe the cached apt data.
6. Run `snapcraft pull` again and notice the download is as in `1.`.
7. Run this test again, but run snapcraft on a partition separated
   from $HOME.
