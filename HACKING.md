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
- Our mailing list is snappy-devel@lists.ubuntu.com
- We can also be found on the #snappy IRC channel on Freenode

### Project Layout

- **bin:** Holds the main snapcraft script. Putting this bin in your PATH or directly running scripts from it will find the rest of the source tree automatically.

- **examples:** Entering the subdirectories and running `../../bin/snapcraft snap` will generally yield interesting results. These examples will give you an idea of what snapcraft can do. These examples are not used during automated testing, they are simply for experimenting.

- **plugins:** Holds yaml metadata for the current snapcraft plugins.

- **tests:** Tests, obviously. `unit` holds Python unit tests and `plainbox` holds plainbox integration tests.

- **snapcraft:** The Python module that houses the core snapcraft logic. The `plugins` subdirectory holds the code for each plugin.
