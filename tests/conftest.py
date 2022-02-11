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

import os

import pytest
import xdg


@pytest.fixture(autouse=True)
def temp_xdg(tmpdir, mocker):
    """Use a temporary locaction for XDG directories."""

    mocker.patch(
        "xdg.BaseDirectory.xdg_config_home", new=os.path.join(tmpdir, ".config")
    )
    mocker.patch("xdg.BaseDirectory.xdg_data_home", new=os.path.join(tmpdir, ".local"))
    mocker.patch("xdg.BaseDirectory.xdg_cache_home", new=os.path.join(tmpdir, ".cache"))
    mocker.patch(
        "xdg.BaseDirectory.xdg_config_dirs", new=[xdg.BaseDirectory.xdg_config_home]
    )
    mocker.patch(
        "xdg.BaseDirectory.xdg_data_dirs", new=[xdg.BaseDirectory.xdg_data_home]
    )
    mocker.patch.dict(os.environ, {"XDG_CONFIG_HOME": os.path.join(tmpdir, ".config")})


@pytest.fixture
def new_dir(tmpdir):
    """Change to a new temporary directory."""

    cwd = os.getcwd()
    os.chdir(tmpdir)

    yield tmpdir

    os.chdir(cwd)
