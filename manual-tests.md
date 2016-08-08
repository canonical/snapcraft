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
4. snapcraft prime
5. ensure that prime/test-owner-file is owned by nobody and nogroup
