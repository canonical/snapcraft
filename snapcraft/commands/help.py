# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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
snapcraft help

Obtain help for plugins and specific topics.

Usage:
  help [options] topics
  help [options] TOPIC
  help [options] PLUGIN

Options:
  -h --help             show this help message and exit.
  -d --devel            show the development help.

To see the list of available plugins run

    snapcraft list-plugins
"""

import importlib
import logging
import sys

# Non-Standard Library modules
from docopt import docopt

# Snapcraft modules
import snapcraft
from snapcraft import sources

logger = logging.getLogger(__name__)

_TOPICS = {
    'sources': sources,
    'plugins': snapcraft,
}


def main(argv=None):
    args = docopt(__doc__, argv=argv)

    topic = args['TOPIC'] or args['PLUGIN']

    if args['topics']:
        for key in _TOPICS:
            print(key)
    elif topic in _TOPICS:
        _topic_help(topic, args['--devel'])
    else:
        _module_help(topic, args['--devel'])


def _topic_help(module_name, devel):
    if devel:
        help(_TOPICS[module_name])
    else:
        print(_TOPICS[module_name].__doc__)


def _module_help(module_name, devel):
    try:
        module = importlib.import_module(
            'snapcraft.plugins.{}'.format(module_name))
        if module.__doc__ and devel:
            help(module)
        elif module.__doc__:
            print(module.__doc__)
        else:
            print('The plugin has no documentation')
    except ImportError:
        logger.error('The plugin does not exist. Run `snapcraft list-plugins` '
                     'to see the available plugins.')
        sys.exit(1)
