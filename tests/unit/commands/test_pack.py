# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2020 Canonical Ltd
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

from testtools.matchers import Equals

from . import LifecycleCommandsBaseTestCase


class TestPack(LifecycleCommandsBaseTestCase):
    def test_using_defaults(self):
        result = self.run_command(["pack", "snap-dir"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_lifecycle_execute.mock.assert_not_called()
        self.fake_pack.mock.assert_called_once_with("snap-dir", output=None)

    def test_output(self):
        result = self.run_command(["pack", "snap-dir", "--output", "foo.snap"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_lifecycle_execute.mock.assert_not_called()
        self.fake_pack.mock.assert_called_once_with("snap-dir", output="foo.snap")
