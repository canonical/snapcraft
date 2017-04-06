# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import snapcraft
from snapcraft.internal import sources

_TOPICS = {
    'sources': sources,
    'plugins': snapcraft,
}


def topic_help(topic, show_devel, list_topics):
    if list_topics:
        for key in _TOPICS:
            print(key)
    elif topic in _TOPICS:
        _topic_help(topic, show_devel)
    else:
        _module_help(topic, show_devel)


def _topic_help(module_name, devel):
    if devel:
        help(_TOPICS[module_name])
    else:
        print(_TOPICS[module_name].__doc__)


def _module_help(module_name, devel):
    try:
        module = importlib.import_module(
            'snapcraft.plugins.{}'.format(module_name.replace("-", "_")))
        if module.__doc__ and devel:
            help(module)
        elif module.__doc__:
            print(module.__doc__)
        else:
            print('The plugin has no documentation')
    except ImportError:
        raise EnvironmentError(
            'The plugin does not exist. Run `snapcraft list-plugins` '
            'to see the available plugins.')
