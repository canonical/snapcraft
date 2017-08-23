# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
import subprocess

from testtools.matchers import Equals

import integration_tests


class NilPluginTestCase(integration_tests.TestCase):

    def test_snap_nil_plugin(self):
        self.run_snapcraft('snap', 'nil-basic')

        dirs = os.listdir(self.prime_dir)
        self.assertThat(dirs, Equals(['meta']))

    def test_nil_no_additional_properties(self):
        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, 'snap',
            'nil-with-additional-properties')

        self.assertTrue(
            "Additional properties are not allowed ('extra-property' was "
            "unexpected)" in exception.output.replace('\n', ' ').strip())
