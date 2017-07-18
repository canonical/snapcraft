# Snapcraft

## Installing from pip

First install a few dependencies:

    sudo apt install gcc g++ make python3-dev python3-venv libffi-dev libsodium-dev libapt-pkg-dev libarchive13 squashfs-tools

Create and activate a new virtual environment:

    mkdir -p ~/venv/snapcraft
    python3 -m venv ~/venv/snapcraft
    source ~/venv/snapcraft/bin/activate


Make sure pip is up-to-date:

    (snapcraft) $ pip install --upgrade pip


Install snapcraft (and its dependencies):

    (snapcraft) $ pip install -r requirements.txt .


## Running

To see all the commands and options, run `snapcraft --help`.


## Hacking

We'd love the help!

- Submit pull requests against [snapcraft](https://github.com/snapcore/snapcraft/pulls)
- Make sure to read the [contribution guide](CONTRIBUTING.md)
- Our mailing list is snapcraft@lists.ubuntu.com
- We can also be found on the #snappy IRC channel on Freenode


### Installing for development

You'll need to install the development dependencies, and you'll also probably want to install it in `editable` mode so any changes you make take affect:

    (snapcraft) $ pip install -r requirements.txt -r requirements-devel.txt --editable .


### Testing

See the [Testing guide](TESTING.md).

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
