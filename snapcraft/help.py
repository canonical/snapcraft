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

import importlib
import logging

import snapcraft
from snapcraft import sources
from snapcraft import cmds


logger = logging.getLogger(__name__)


_TOPICS = {
    'sources': sources,
    'plugins': snapcraft,
}


def topic(args=None):
    """Get help on additional topics or plugin usage.
    snapcraft help topics    To get the list of topics
    snapcraft help <plugin>  To get help for a specific plugin
    snapcraft help <topic>   To get help on a speficic topic

To see the list of available plugins run

    snapcraft list-plugins
"""
    if args.topic == 'topics':
        for key in _TOPICS:
            print(key)
    elif args.topic in _TOPICS:
        _topic_help(args.topic, args.devel)
    else:
        _module_help(args.topic, args.devel)


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
        logger.error('The plugin does not exist. Use one of the following:')
        cmds.list_plugins()
