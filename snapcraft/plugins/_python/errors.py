# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
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

import snapcraft.internal.errors
import snapcraft.formatting_utils


class PythonPluginError(snapcraft.internal.errors.SnapcraftError):
    pass


class MissingPythonCommandError(PythonPluginError):

    fmt = 'Unable to find {python_version}, searched: {search_paths}'

    def __init__(self, python_version, search_paths):
        super().__init__(
            python_version=python_version,
            search_paths=snapcraft.formatting_utils.combine_paths(
                search_paths, '', ':'))
