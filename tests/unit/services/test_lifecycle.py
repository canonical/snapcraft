# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Tests for the Snapcraft Lifecycle service."""


def test_lifecycle_installs_base(lifecycle_service, mocker):
    install_snaps = mocker.patch("craft_parts.packages.snaps.install_snaps")

    lifecycle_service.setup()
    lifecycle_service.run("pull")

    info = lifecycle_service.project_info
    assert info.base == "core24"
    install_snaps.assert_called_once_with(
        {"core24"},
    )
