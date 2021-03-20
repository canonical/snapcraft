# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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
import pathlib
from typing import List
from unittest import mock

import pytest
import xdg


def pytest_generate_tests(metafunc):
    idlist = []
    argvalues = []
    if metafunc.cls is None:
        return

    for scenario in metafunc.cls.scenarios:
        idlist.append(scenario[0])
        items = scenario[1].items()
        argnames = [x[0] for x in items]
        argvalues.append([x[1] for x in items])
    metafunc.parametrize(argnames, argvalues, ids=idlist, scope="class")


@pytest.fixture
def mock_subprocess_run():
    """A no-op subprocess.run mock."""
    patcher = mock.patch("subprocess.run")
    yield patcher.start()
    patcher.stop()


@pytest.fixture
def tmp_work_path(tmp_path):
    """Setup a temporary directory and chdir to it."""
    os.chdir(tmp_path)
    return tmp_path


@pytest.fixture
def xdg_dirs(tmp_path, monkeypatch):
    """Setup XDG directories in a temporary directory."""
    monkeypatch.setattr(
        xdg.BaseDirectory, "xdg_config_home", (tmp_path / ".config").as_posix()
    )
    monkeypatch.setattr(
        xdg.BaseDirectory, "xdg_data_home", (tmp_path / ".local").as_posix()
    )
    monkeypatch.setattr(
        xdg.BaseDirectory, "xdg_cache_home", (tmp_path / ".cache").as_posix()
    )
    monkeypatch.setattr(
        xdg.BaseDirectory,
        "xdg_config_dirs",
        lambda: [(tmp_path / ".config").as_posix()],
    )
    monkeypatch.setattr(
        xdg.BaseDirectory, "xdg_data_dirs", lambda: [(tmp_path / ".config").as_posix()]
    )

    monkeypatch.setenv("XDG_CONFIG_HOME", (tmp_path / ".config").as_posix())
    monkeypatch.setenv("XDG_DATA_HOME", (tmp_path / ".local").as_posix())
    monkeypatch.setenv("XDG_CACHE_HOME", (tmp_path / ".cache").as_posix())

    return tmp_path


@pytest.fixture()
def in_snap(monkeypatch):
    """Simualte being run from within the context of the Snapcraft snap."""
    monkeypatch.setenv("SNAP", "/snap/snapcraft/current")
    monkeypatch.setenv("SNAP_NAME", "snapcraft")
    monkeypatch.setenv("SNAP_VERSION", "4.0")


@pytest.fixture()
def fake_exists(monkeypatch):
    """Fakely return True when checking for preconfigured paths."""

    class FileCheck:
        def __init__(self) -> None:
            self._original_exists = os.path.exists
            self.paths: List[str] = list()

        def exists(self, path: str) -> bool:
            if pathlib.Path(path) in self.paths:
                return True
            return self._original_exists(path)

    file_checker = FileCheck()
    monkeypatch.setattr(os.path, "exists", file_checker.exists)

    return file_checker
