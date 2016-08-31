# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import integration_tests

from testtools.matchers import (
    DirExists,
    FileExists,
    Not,
)


class RpathTestCase(integration_tests.TestCase):

    def test_origin_rpath(self):
        project_dir = 'rpath-test'
        self.run_snapcraft('prime', project_dir)

        primedir = os.path.join(project_dir, 'prime')
        self.assertThat(os.path.join(primedir, 'binary'), FileExists())
        self.assertThat(
            os.path.join(primedir, 'lib', 'libfoo.so'), FileExists())

        # Assert that the $ORIGIN rpath did not result in the library being
        # pulled in twice.
        self.assertThat(
            os.path.join(primedir, os.path.abspath(primedir).lstrip('/')),
            Not(DirExists()))
