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

import yaml

from snapcraft.internal.states._state import State


def _stage_state_constructor(loader, node):
    parameters = loader.construct_mapping(node)
    return StageState(**parameters)

yaml.add_constructor(u'!StageState', _stage_state_constructor)


class StageState(State):
    yaml_tag = u'!StageState'

    def __init__(self, files, directories, options=None, project=None):
        super().__init__(options, project)

        self.files = files
        self.directories = directories

    def properties_of_interest(self, options):
        """Extract the properties concerning this step from the options.

        The only property of interest to the stage step is the `stage` keyword
        used to filter out files with a white or blacklist.
        """

        return {'stage': getattr(options, 'stage', ['*']) or ['*']}

    def project_options_of_interest(self, project):
        """Extract the options concerning this step from the project.

        The stage step doesn't care about any project options.
        """

        return {}
