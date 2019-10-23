# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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


class TestDir(fixtures.Fixture):
    def _setUp(self):
        self.path = self.useFixture(fixtures.TempDir()).path

    def create_file(self, *name: str) -> None:
        open(os.path.join(self.path, *name), "wb").close()

    def create_dir(self, *name: str) -> None:
        os.mkdir(os.path.join(self.path, *name))

    def unlink(self, *name) -> None:
        os.unlink(os.path.join(self.path, *name))

    def rmdir(self, *name: str) -> None:
        os.rmdir(os.path.join(self.path, *name))

    def exists(self, *parts) -> bool:
        return os.path.exists(os.path.join(self.path, *parts))


class Chdir:
    def __init__(self, path):
        self._path = os.path.expanduser(path)

    def __enter__(self):
        self._old_path = os.getcwd()
        os.chdir(self._path)

    def __exit__(self, etype, value, traceback):
        os.chdir(self._old_path)
