# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from typing import Any, Dict


class ProjectInfo:
    """Information gained from the snap's snapcraft.yaml file."""

    def __init__(self, data: Dict[str, Any]) -> None:
        self.name = data['name']
        self.version = data.get('version')
        self.summary = data.get('summary')
        self.description = data.get('description')
        self.confinement = data.get('confinement')
