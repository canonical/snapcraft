# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

from unittest import mock
from testtools.matchers import FileExists

from snapcraft.internal.sources import Script
from tests import unit


class TestScript(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.source = Script("source", "destination")

        os.mkdir("destination")
        self.source.file = os.path.join("destination", "file")
        open(self.source.file, "w").close()

    @mock.patch("snapcraft.internal.sources._script.FileBase.download")
    def test_download_makes_executable(self, mock_download):
        self.source.file = os.path.join("destination", "file")
        self.source.download()
        self.assertThat(self.source.file, FileExists())
        self.assertThat(self.source.file, unit.IsExecutable())
