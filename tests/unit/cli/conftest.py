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


@pytest.fixture
def mock_echo_error():
    """Return a mock for snapcraft.cli.echo.error."""
    patcher = mock.patch("snapcraft.cli.echo.error")
    yield patcher.start()
    patcher.stop()


@pytest.fixture
def mock_sys_exit():
    """Return a mock for sys.exit."""
    patcher = mock.patch("sys.exit")
    yield patcher.start()
    patcher.stop()
