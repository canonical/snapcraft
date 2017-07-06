# Snapcraft tests

## Manual tests

Snapcraft has a few manual tests documented in [manual-tests.md](manual-tests.md).
These are tests for features that are not possible to automate, or that the complexity of automating them is too high while the time it takes to run them by hand is not too much.

We try very hard to automate as much as possible and to keep the manual test suite as small as possible. Please do not add a manual test before discussing it with the team, and make sure that there are very good reasons not to automate it.

### Staging server

Snapcraft has the ability to upload snaps for publication in the Snappy Store. If you're working on a feature that requires you to interact with the store, you might want to use the staging server instead of the production store. To do
that, make sure you have an account on the [staging server](https://login.staging.ubuntu.com), then run:

    source tools/staging_env.sh

You will see a prompt indicating that you are going to be talking to the staging server. Once you are done working with the staging servers you can run `deactivate`.

## Automated tests in the snapcraft repository

The snapcraft repository has multiple suites of automated tests which run the code at different levels, following different principles of what to execute, how to force different code paths, and how to verify the results.

### Static tests

The static tests suite performs a static analysis on the source code without executing it. It will catch syntax and code style errors.

### Unit tests

The unit tests is a suite of low-level white box tests. They excercise units of code to verify that the different parts work as expected in a fully isolated way. Ideally, these tests should call only public functions, objects and methods, leaving the internals of snapcraft as implementation details that can change without having to modifying any tests. To isolate the units from their environment and dependencies we can replace those with test doubles. In order to set up test doubles, we prefer dependency injection through arguments than excessive mocking. These tests can verify the results checking the output printed to the command line, checking the files created during the excecution, inspecting the calls made to the test doubles and verifying that the expected exceptions were thrown.

These tests are in the `snapcraft/tests` directory.

### Integration tests

The integration tests are a group of suites that excercise snapcraft as a black box. They are only allowed to set up the environment where snapcraft runs and create files; but for the execution phase of the test they can only run the snapcraft command or one of its subcommands. To verify the results they can check the output printed to the command line, the return value of the snapcraft command, and any files created during the execution.

This suite was split in three: plugins, store and other integration tests. This split is artificial, we made it just because the full suite takes more time than what Travis allows for a single job.

These tests are in the `integration_tests` directory, with the `snapcraft.yamls` and other source files for the tests snaps in `integration_tests/snaps`.

### Snaps tests

The snaps tests is a suite of high-level tests that try to simulate real-world scenarios of a user interacting with snapcraft. They cover the call to snapcraft to generate a snap file from the source files of a fully functional project, the installation of the resulting snap, and the execution of the binaries and services of this snap.

These tests are in the `snaps_tests` directory, with the sources for the test snaps in the `demos` directory.

### Setting up the environment

In order to run these tests suites, first you will need to set up your development environment. Follow the steps in the [Hacking guide](HACKING.md) to install for development.

Then, you'll need a few more dependencies:

    sudo apt install squashfs-tools xdelta3 bzr git mercurial subversion

### Running the tests

To run all the tests execute:

    ./runtests.sh

You can selectively run only one of the suites, and apply a filter:

    ./runtests.sh [static|unit|integration|plugins|store|snaps] [pattern]

Examples:

  * To run only the full unit test suite:

    ```
    ./runtests.sh unit
    ```

  * To run only the unit tests in the file called `test_init.py`:

    ```
    ./runtests.sh unit test_init.py

  * To run only the integration tests that interact with the store:

    ```
    ./runtests.sh store
    ```

  * To run only the integration tests for the python plugin:

    ```
    ./runtests.sh plugins *python*
    ```
