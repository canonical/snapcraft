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

from unittest import mock

import pytest

from snapcraft_legacy.internal.meta.snap import Snap
from snapcraft_legacy.project import Project


@pytest.fixture
def project(monkeypatch, tmp_work_path, request):
    """Return project variants for core and core18"""
    monkeypatch.setattr(Project, "parallel_build_count", 2)

    snapcraft_project = Project()
    snapcraft_project._snap_meta = Snap(
        name="test-snap", base="core18", confinement="strict"
    )
    return snapcraft_project


@pytest.fixture
def mock_common_run_output():
    """A no-op common.run_output mock."""
    patcher = mock.patch("snapcraft_legacy.internal.common.run_output")
    yield patcher.start()
    patcher.stop()


@pytest.fixture
def mock_run():
    """A no-op run mock."""
    patcher = mock.patch("snapcraft_legacy.plugins.v1.PluginV1.run")
    yield patcher.start()
    patcher.stop()


@pytest.fixture
def mock_run_output():
    """A no-op run_output mock."""
    patcher = mock.patch("snapcraft_legacy.plugins.v1.PluginV1.run_output")
    yield patcher.start()
    patcher.stop()


@pytest.fixture
def mock_tar():
    """A no-op tar source mock."""
    patcher = mock.patch("snapcraft_legacy.internal.sources.Tar")
    yield patcher.start()
    patcher.stop()


@pytest.fixture
def mock_zip():
    """A no-op zip source mock."""
    patcher = mock.patch("snapcraft_legacy.internal.sources.Zip")
    yield patcher.start()
    patcher.stop()
