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


# Test cleanbuild with debug shell

1. Run `snapcraft cleanbuild --debug` for a snap.
2. Insert an error such that the code will fail to compile or
   mistype the name of an entry in `stage-packages`.
3. Run `snapcraft cleanbuild --debug` again.
4. Ensure you are dropped into a debug shell.
5. Exit the shell.
6. Ensure you are dropped back into your original shell session.


# Test cleanbuild with a remote.

1. Setup a remote as described on
   https://linuxcontainers.org/lxd/getting-started-cli/#multiple-hosts
2. Select a project to build.
3. Run `snapcraft cleanbuild --remote <remote>` where `<remote>` is
   the name you gave the remote on step 1.
