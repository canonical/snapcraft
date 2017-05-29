# Snapcraft

## Installing from pip

First install a few dependencies:

    sudo apt install gcc g++ make python3-venv libffi-dev libsodium-dev libapt-pkg-dev libarchive13

Create and activate a new virtual environment:

    python3 -m venv path/to/venv
    source path/to/venv/bin/activate


Make sure pip is up-to-date:

    (venv) $ pip install --upgrade pip


Install snapcraft (and its dependencies):

    (venv) $ pip install -r requirements.txt .


## Running

To see all the commands and options, run `snapcraft --help`.


## Hacking

We'd love the help!

- Submit pull requests against [snapcraft](https://github.com/snapcore/snapcraft/pulls)
- Make sure to read the [contribution guide](CONTRIBUTING.md)
- Our mailing list is snapcraft@lists.ubuntu.com
- We can also be found on the #snappy IRC channel on Freenode


### Installing for development

You'll need to install the development dependencies, and you'll also probably want to install it in `editable` mode so your changes actually do something:

    (venv) $ pip install -r requirements.txt -r requirements-devel.txt --editable .


### Testing

You'll need a few more dependencies in order to run the tests:

    sudo apt install squashfs-tools xdelta3 bzr git mercurial subversion

To run all the tests execute:

    ./runtests.sh

You can selectively run a selective group of tests like:

    ./runtests.sh [static|unit|integration|snaps]

The test groups mean:

- static: static code analysis done with flake8
- unit: run the standard unit tests
- integration: shells to snapcraft, builds
- snaps: builds snaps, installs and checks how they run


### Staging server

Snapcraft has the ability to upload snaps for publication in the Snappy Store.
If you're working on a feature that requires you to interact with the store, you
might want to use the staging server instead of the production store. To do
that, make sure you have an account on the
[staging server](https://login.staging.ubuntu.com), then run:

    source tools/staging_env.sh

You will see a prompt indicating that you are going to be talking to the staging
server. Once you are done working with the staging servers you can run `deactivate`.


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
