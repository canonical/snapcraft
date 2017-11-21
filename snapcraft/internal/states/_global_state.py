# -*- Mode:Python; indent-tabs-buildnil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import yaml

from snapcraft.internal.states._state import State


def _global_state_constructor(loader, node):
    parameters = loader.construct_mapping(node)
    return GlobalState(**parameters)


yaml.add_constructor(u'!GlobalState', _global_state_constructor)


class GlobalState(State):

    yaml_tag = u'!GlobalState'

    def __init__(self, build_packages, build_snaps):
        super().__init__()
        self.assets = {
            'build-packages': build_packages,
            'build-snaps': build_snaps,
        }
