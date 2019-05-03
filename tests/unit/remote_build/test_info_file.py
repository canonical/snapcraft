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

import fixtures
import os

from snapcraft.internal.remote_build import InfoFile
from tests import unit
from testtools.matchers import Equals, FileExists


class TestInfoFile(unit.TestCase):
    def setUp(self):
        super().setUp()
        self._path = self.useFixture(fixtures.TempDir()).path

    def test_creation(self):
        path = os.path.join(self._path, "test.yaml")
        info = InfoFile(path)
        info.save()
        self.assertThat(path, FileExists())

    def test_save(self):
        path = os.path.join(self._path, "test.yaml")
        info = InfoFile(path)
        info.save(foo="bar")
        with open(path, "r") as f:
            data = f.readlines()
        self.assertThat(data, Equals(["foo: bar\n"]))

    def test_load(self):
        path = os.path.join(self._path, "test.yaml")
        with open(path, "w") as f:
            f.write("foo: 1\nbar:\n- a\n- b\n")
        info = InfoFile(path)
        info.load()
        self.assertThat(info["foo"], Equals(1))
        self.assertThat(info["bar"], Equals(["a", "b"]))
