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

import shelve
import pathlib
import time
from testtools.matchers import Equals

from snapcraft.internal.pcache import pcache
from tests import unit


class TestPcacheScenarios(unit.TestCase):
    scenarios = [
        ("immediate expiry", dict(expiry_secs=0, sleep_secs=0, increments=True)),
        ("expired", dict(expiry_secs=0.10, sleep_secs=0.11, increments=True)),
        ("not expired", dict(expiry_secs=3600, sleep_secs=0, increments=False)),
    ]

    def test_run(self):
        run_ntimes = 0

        @pcache(
            path_func=lambda: pathlib.Path(self.path, "cache"),
            expiry_secs=self.expiry_secs,
        )
        def add(a, b, *, c):
            nonlocal run_ntimes
            run_ntimes += 1
            return a + b + c

        for i in range(1, 5):
            result = add(1, 2, c=3)
            self.assertThat(result, Equals(6))

            if self.increments is True:
                self.assertThat(run_ntimes, Equals(i))
            else:
                self.assertThat(run_ntimes, Equals(1))

            if self.sleep_secs > 0:
                time.sleep(self.sleep_secs)


class TestIncompatibleDb(unit.TestCase):
    def test_corrupt_db(self):
        path = pathlib.Path(self.path, "cache")
        path.write_text("corrupt")
        run_ntimes = 0

        @pcache(path_func=lambda: path)
        def add(a, b, *, c):
            nonlocal run_ntimes
            run_ntimes += 1
            return a + b + c

        for i in range(1, 2):
            result = add(1, 2, c=3)
            self.assertThat(result, Equals(6))
            self.assertThat(run_ntimes, Equals(1))

    def test_incompatible_db(self):
        path = pathlib.Path(self.path, "cache")

        with shelve.open(str(path), writeback=True) as db:
            db["foo"] = "bar"

        run_ntimes = 0

        @pcache(path_func=lambda: path)
        def add(a, b, *, c):
            nonlocal run_ntimes
            run_ntimes += 1
            return a + b + c

        for i in range(1, 2):
            result = add(1, 2, c=3)
            self.assertThat(result, Equals(6))
            self.assertThat(run_ntimes, Equals(1))


class TestCreation(unit.TestCase):
    def test_no_parent_dir(self):
        path = pathlib.Path(self.path, "missing", "x.cache")

        @pcache(path_func=lambda: path)
        def add(a, b, *, c):
            return a + b + c

        result = add(1, 2, c=3)
        self.assertThat(result, Equals(6))
        self.assertThat(path.exists(), Equals(True))
