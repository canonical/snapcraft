#!/usr/bin/env python3
#
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2025 Canonical Ltd.
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

"""Creation of schema for snapcraft.yaml."""

if __name__ == "__main__":
    import json

    import pydantic

    # The TypeAdapter complains if we don't have this.
    from snapcraft.models.app import App  # noqa
    from snapcraft.models.project import SnapcraftProject

    print(json.dumps(pydantic.TypeAdapter(SnapcraftProject).json_schema(), indent="  "))
