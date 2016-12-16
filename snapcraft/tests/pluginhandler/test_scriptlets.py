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

from testtools.matchers import FileExists

from . import mocks
from snapcraft import tests


class ScriptletTestCase(tests.TestCase):

    def test_run_prepare_scriptlet(self):
        handler = mocks.loadplugin(
            'test-part', part_properties={'prepare': 'touch before-build'})

        handler.build()

        before_build_file_path = os.path.join(handler.code.build_basedir,
                                              'before-build')
        self.assertThat(before_build_file_path, FileExists())

    def test_run_install_scriptlet(self):
        handler = mocks.loadplugin(
            'test-part', part_properties={'install': 'touch after-build'})

        handler.build()

        after_build_file_path = os.path.join(handler.code.build_basedir,
                                             'after-build')
        self.assertThat(after_build_file_path, FileExists())
