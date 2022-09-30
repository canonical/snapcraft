# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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


from snapcraft.providers import providers


def test_get_instance_name(new_dir):
    """Test formatting of instance name."""
    inode_number = str(new_dir.stat().st_ino)
    expected_name = f"snapcraft-hello-world-on-arm64-for-armhf-{inode_number}"
    actual_name = providers.get_instance_name(
        project_name="hello-world",
        project_path=new_dir,
        build_on="arm64",
        build_for="armhf",
    )
    assert expected_name == actual_name
