# Snapcraft

## Running

To see all the commands and options, run `snapcraft --help`.

## Testing

Simply run the top level testing script:

    ./runtests.sh

- If you want to get a test coverage report, install python3-coverage before running the tests:

    sudo apt-get install python3-coverage


- If you don't want to run the plainbox integration tests, you can skip them by setting SNAPCRAFT_TESTS_SKIP_PLAINBOX=1 in your environment.

- If you are on 15.04 or earlier, you will need to run:

    sudo add-apt-repository ppa:hardware-certification/public

### PPA

You can install the daily build PPA by running:

    sudo add-apt-repository ppa:snappy-dev/snapcraft-daily

## Hacking

We'd love the help!

- Submit pull requests against [snapcraft](https://github.com/ubuntu-core/snapcraft/pulls)
- Make sure to read the [contribution guide](CONTRIBUTING.md)
- Our mailing list is snappy-devel@lists.ubuntu.com
- We can also be found on the #snappy IRC channel on Freenode


### Staging server

Snapcraft has the ability to upload snaps for publication in the Snappy Store.
If you're working on a feature that requires you to interact with the store, you
might want to use the staging server instead of the production store. To do
that, make sure you have an account on the
[staging server](https://login.staging.ubuntu.com), then set the following
environment variables:

    UBUNTU_STORE_API_ROOT_URL='https://myapps.developer.staging.ubuntu.com/dev/api/'
    UBUNTU_STORE_UPLOAD_ROOT_URL='https://upload.apps.staging.ubuntu.com/'
    UBUNTU_SSO_API_ROOT_URL='https://login.staging.ubuntu.com/api/v2/'


### Project Layout

- **bin:** Holds the main snapcraft script. Putting this bin in your PATH or directly running scripts from it will find the rest of the source tree automatically.

- **examples:** Entering the subdirectories and running `../../bin/snapcraft snap` will generally yield interesting results. These examples will give you an idea of what snapcraft can do. These examples are not used during automated testing, they are simply for experimenting.

- **plugins:** Holds yaml metadata for the current snapcraft plugins.

- **tests:** Tests, obviously. `unit` holds Python unit tests and `plainbox` holds plainbox integration tests.

- **snapcraft:** The Python module that houses the core snapcraft logic. The `plugins` subdirectory holds the code for each plugin.

### Updating library filter

To update the list of libraries that get excluded from inclusion into a
snap run:

    ./libraries/generate_lib_list.py libraries/<release>

e.g.; to update the list for 16.04,

    ./libraries/generate_lib_list.py libraries/16.04
