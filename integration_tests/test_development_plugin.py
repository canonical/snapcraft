# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

from testtools.matchers import (
    DirExists,
    FileExists
)

import integration_tests


class DevelopmentPluginTestCase(integration_tests.TestCase):

    def test_stage_developer_plugin(self):
        project_dir = 'simple-development'
        self.run_snapcraft('stage', project_dir)

        expected_files = [
            'libapitest.c',
            'libapitest.h',
            'libapitest.o',
            'libapitest.so',
            'libapitest.so.1',
            'libapitest.so.1.0.1',
            'main.c',
            'Makefile',
        ]
        for expected_file in expected_files:
            self.assertThat(
                os.path.join(project_dir, 'stage', expected_file),
                FileExists())
        self.assertThat(
            'libapitest-api.tar.gz',
            FileExists())
