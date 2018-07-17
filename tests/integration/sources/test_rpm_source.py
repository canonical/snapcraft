# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Neal Gompa
# Copyright (C) 2017-2018 Canonical Ltd
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

from testtools.matchers import FileExists

from tests import integration


class RpmSourceTestCase(integration.TestCase):
    def test_stage_rpm(self):
        self.run_snapcraft("stage", "rpm-hello")

        self.assertThat(os.path.join(self.stage_dir, "bin", "hello"), FileExists())
        self.assertThat(
            os.path.join(self.stage_dir, "usr", "bin", "world"), FileExists()
        )
