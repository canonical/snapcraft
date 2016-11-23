#!/usr/bin/python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
snapcraft

Usage:
  snapcraft [options] [--enable-geoip --no-parallel-build]
  snapcraft [options] init
  snapcraft [options] pull [<part> ...]  [--enable-geoip]
  snapcraft [options] build [<part> ...] [--no-parallel-build]
  snapcraft [options] stage [<part> ...]
  snapcraft [options] prime [<part> ...]
  snapcraft [options] strip [<part> ...]
  snapcraft [options] clean [<part> ...] [--step <step>]
  snapcraft [options] snap [<directory> --output <snap-file>]
  snapcraft [options] cleanbuild
  snapcraft [options] login
  snapcraft [options] logout
  snapcraft [options] list-keys
  snapcraft [options] keys
  snapcraft [options] create-key [<key-name>]
  snapcraft [options] register-key [<key-name>]
  snapcraft [options] register <snap-name> [--private]
  snapcraft [options] sign-build <snap-file> [--key-name=<key-name>] [--local]
  snapcraft [options] upload <snap-file>
  snapcraft [options] push <snap-file> [--release <channels>]
  snapcraft [options] release <snap-name> <revision> <channel>
  snapcraft [options] status <snap-name> [--series=<series>] [--arch=<arch>]
  snapcraft [options] history <snap-name> [--series=<series>] [--arch=<arch>]
  snapcraft [options] close <snap-name> <channel_names>...
  snapcraft [options] list-plugins
  snapcraft [options] tour [<directory>]
  snapcraft [options] update
  snapcraft [options] gated <snap-name>
  snapcraft [options] validate <snap-name> <validation>... [--key-name=<key-name>]
  snapcraft [options] define <part-name>
  snapcraft [options] search [<query> ...]
  snapcraft [options] enable-ci [<ci-system>]
  snapcraft [options] help (topics | <plugin> | <topic>) [--devel]
  snapcraft (-h | --help)
  snapcraft --version

Options:
  -h --help                             show this help message and exit
  -v --version                          show program version and exit
  -d --debug                            print debug information while executing
                                        (including backtraces). When used with
                                        cleanbuild, it opens a shell in case
                                        of failure.
  --target-arch ARCH                    EXPERIMENTAL: sets the target
                                        architecture. Very few plugins support
                                        this.

Options specific to pulling:
  --enable-geoip         enables geoip for the pull step if stage-packages
                         are used.

Options specific to building:
  --no-parallel-build                   use only a single build job per part
                                        (the default number of jobs per part is
                                        equal to the number of CPUs)

Options specific to cleaning:
  -s <step>, --step <step>              only clean the specified step and those
                                        that depend upon it. <step> can be one
                                        of: pull, build, stage or prime.

Options specific to snapping:
  -o <snap-file>, --output <snap-file>  used in case you want to rename the
                                        snap.

Options specific to store interaction:
  --release <channels>  Comma separated list of channels to release to.
  --series <series>     Snap series [default: {DEFAULT_SERIES}].

The available commands are:
  help         Obtain help for a certain plugin or topic
  init         Initialize a snapcraft project.
  list-plugins List the available plugins that handle different types of part.
  login        Authenticate session against Ubuntu One SSO.
  logout       Clear session credentials.
  list-keys    List keys available for signing snaps.
  keys         Alias for list-keys.
  create-key   Create a key pair for signing snaps.
  register-key Register a key for signing snaps.
  register     Register the package name in the store.
  tour         Setup the snapcraft examples tour in the specified directory,
               or ./snapcraft-tour/.
  sign-build   Sign a built snap file and assert it using the developer's key.
  push         Pushes and optionally releases a snap to the Ubuntu Store.
  upload       DEPRECATED Upload a snap to the Ubuntu Store. The push command
               supersedes this command.
  release      Release a revision of a snap to a specific channel.
  status       Show the current status of a snap per channel and architecture.
  history      List all revisions of a snap.
  close        Close one or more channels of a snap.
  enable-ci    EXPERIMENTAL enable continuous-integration systems to build and
               release snaps to the Ubuntu Store.
The available lifecycle commands are:
  clean        Remove content - cleans downloads, builds or install artifacts.
  cleanbuild   Create a snap using a clean environment managed by lxd.
  pull         Download or retrieve artifacts defined for a part.
  build        Build artifacts defined for a part. Build systems capable of
               running parallel build jobs will do so unless
               "--no-parallel-build" is specified.
  stage        Stage the part's built artifacts into the common staging area.
  prime        Final copy and preparation for the snap.
  snap         Create a snap.

Parts ecosystem commands:
  update       Updates the parts listing from the cloud.
  define       Shows the definition for the cloud part.
  search       Searches the remote parts cache for matching parts.

Calling snapcraft without a COMMAND will default to 'snap'

The cleanbuild command requires a properly setup lxd environment that
can connect to external networks. Refer to the "Ubuntu Desktop and
Ubuntu Server" section on
https://linuxcontainers.org/lxd/getting-started-cli
to get started.

For more help, visit the documentation:
http://snapcraft.io/docs/build-snaps
"""  # NOQA

import logging
import os
import pkgutil
import shutil
import sys

from docopt import docopt

import snapcraft
from snapcraft.integrations import enable_ci
from snapcraft.internal import lifecycle, log, parts
from snapcraft.internal.common import (
    format_output_in_columns,
    get_terminal_width,
    get_tourdir)
from snapcraft.storeapi.constants import DEFAULT_SERIES


logger = logging.getLogger(__name__)
_SNAPCRAFT_TOUR_DIR = "./snapcraft-tour/"


def _scaffold_examples(directory):
    logger.debug("Copying examples tour to {}".format(directory))
    dest_dir = os.path.abspath(directory)

    # If dest_dir doesn't exist, we dump all examples in it.
    # If it does exist, we dump them into a subdirectory if it's not
    # the default dir.
    try:
        shutil.copytree(get_tourdir(), dest_dir)
    except FileExistsError:
        # default crafted directory shouldn't add itself inside
        if directory == _SNAPCRAFT_TOUR_DIR:
            raise FileExistsError("{} already exists, please specify a "
                                  "destination directory.".format(directory))
        # don't event try to copy if the dest exists already
        if not os.path.isdir(dest_dir):
            raise NotADirectoryError("{} is a file, can't be used as a "
                                     "destination".format(dest_dir))
        dest_dir = os.path.normpath(os.path.join(dest_dir,
                                                 _SNAPCRAFT_TOUR_DIR))
        shutil.copytree(get_tourdir(), dest_dir)

    print("Snapcraft tour initialized in {}\n"
          "Instructions are in the README, or "
          "http://snapcraft.io/create/#tour".format(directory))


def _list_plugins():
    plugins = []
    for importer, modname, is_package in pkgutil.iter_modules(
            snapcraft.plugins.__path__):
        plugins.append(modname.replace('_', '-'))

    # we wrap the output depending on terminal size
    width = get_terminal_width()
    for line in format_output_in_columns(plugins, max_width=width):
        print(line)


def _get_project_options(args):
    options = {}
    options['use_geoip'] = args['--enable-geoip']
    options['parallel_builds'] = not args['--no-parallel-build']
    options['target_deb_arch'] = args['--target-arch']
    options['debug'] = args['--debug']

    return snapcraft.ProjectOptions(**options)


def main(argv=None):
    doc = __doc__.format(DEFAULT_SERIES=DEFAULT_SERIES)
    args = docopt(doc, version=snapcraft.__version__, argv=argv)

    # Default log level is INFO unless --debug is specified
    log_level = logging.INFO
    if args['--debug']:
        log_level = logging.DEBUG

    log.configure(log_level=log_level)
    project_options = _get_project_options(args)

    if args['strip']:
        logger.warning("DEPRECATED: use 'prime' instead of 'strip'")
        args['prime'] = True
    try:
        return run(args, project_options)
    except Exception as e:
        if args['--debug']:
            raise

        logger.error(str(e))
        sys.exit(1)


def _get_lifecycle_command(args):
    lifecycle_commands = ['pull', 'build', 'stage', 'prime']
    lifecycle_command = [k for k in lifecycle_commands if args[k]]
    if len(lifecycle_command) == 0:
        return None
    return lifecycle_command[0]


def _get_command_from_arg(args):
    functions = {
        'init': lifecycle.init,
        'login': snapcraft.login,
        'logout': snapcraft.logout,
        'list-plugins': _list_plugins,
    }
    function = [k for k in functions if args[k]]
    if len(function) == 0:
        return None
    return functions[function[0]]


def run(args, project_options):  # noqa
    lifecycle_command = _get_lifecycle_command(args)
    argless_command = _get_command_from_arg(args)
    if lifecycle_command:
        lifecycle.execute(
            lifecycle_command, project_options, args['<part>'])
    elif argless_command:
        argless_command()
    elif args['clean']:
        _run_clean(args, project_options)
    elif args['cleanbuild']:
        lifecycle.cleanbuild(project_options),
    elif _is_store_command(args):
        _run_store_command(args)
    elif args['tour']:
        _scaffold_examples(args['<directory>'] or _SNAPCRAFT_TOUR_DIR)
    elif args['help']:
        snapcraft.topic_help(args['<topic>'] or args['<plugin>'],
                             args['--devel'], args['topics'])
    elif args['enable-ci']:
        enable_ci(args['<ci-system>'])
    elif args['update']:
        parts.update()
    elif args['define']:
        parts.define(args['<part-name>'])
    elif args['search']:
        parts.search(' '.join(args['<query>']))
    else:  # snap by default:
        lifecycle.snap(project_options, args['<directory>'], args['--output'])

    return project_options


def _run_clean(args, project_options):
    step = args['--step']
    if step == 'strip':
        logger.warning('DEPRECATED: Use `prime` instead of `strip` '
                       'as the step to clean')
        step = 'prime'
    lifecycle.clean(project_options, args['<part>'], step)


def _is_store_command(args):
    commands = (
        'list-keys', 'keys', 'create-key', 'register-key', 'register',
        'sign-build', 'upload', 'release', 'push', 'validate', 'gated',
        'history', 'status', 'close')
    return any(args.get(command) for command in commands)


# This function's complexity is correlated to the number of
# commands, no point in checking that.
def _run_store_command(args):  # noqa: C901
    if args['list-keys'] or args['keys']:
        snapcraft.list_keys()
    elif args['create-key']:
        snapcraft.create_key(args['<key-name>'])
    elif args['register-key']:
        snapcraft.register_key(args['<key-name>'])
    elif args['register']:
        snapcraft.register(args['<snap-name>'], args['--private'])
    elif args['sign-build']:
        snapcraft.sign_build(
            args['<snap-file>'], args['--key-name'], args['--local'])
    elif args['upload']:
        logger.warning('DEPRECATED: Use `push` instead of `upload`')
        snapcraft.push(args['<snap-file>'])
    elif args['push']:
        if args['--release']:
            release_channels = args['--release'].split(',')
        else:
            release_channels = []
        snapcraft.push(args['<snap-file>'], release_channels)
    elif args['release']:
        snapcraft.release(
            args['<snap-name>'], args['<revision>'], [args['<channel>']])
    elif args['validate']:
        snapcraft.validate(args['<snap-name>'], args['<validation>'],
                           key=args['--key-name'])
    elif args['gated']:
        snapcraft.gated(args['<snap-name>'])
    elif args['status']:
        snapcraft.status(
            args['<snap-name>'], args['--series'], args['--arch'])
    elif args['history']:
        snapcraft.history(
            args['<snap-name>'], args['--series'], args['--arch'])
    elif args['close']:
        snapcraft.close(args['<snap-name>'], args['<channel_names>'])


if __name__ == '__main__':  # pragma: no cover
    main()                  # pragma: no cover
