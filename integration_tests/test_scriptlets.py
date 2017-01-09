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

from testtools.matchers import FileContains, FileExists


class ScriptletTestCase(integration_tests.TestCase):

    def test_prepare_scriptlet(self):
        project_dir = 'scriptlet-prepare'
        self.run_snapcraft('build', project_dir)

        installdir = os.path.join(
            project_dir, 'parts', 'prepare-scriptlet-test', 'install')
        touch_file_path = os.path.join(installdir, 'prepared')
        self.assertThat(touch_file_path, FileExists())

    def test_build_scriptlet(self):
        project_dir = 'scriptlet-build'
        self.run_snapcraft('build', project_dir)

        partdir = os.path.join(project_dir, 'parts', 'build-scriptlet-test')
        builddir = os.path.join(partdir, 'build')
        installdir = os.path.join(partdir, 'install')

        touch_file_path = os.path.join(builddir, 'build-build')
        self.assertThat(touch_file_path, FileExists())
        touch_file_path = os.path.join(installdir, 'build-install')
        self.assertThat(touch_file_path, FileExists())

    def test_install_scriptlet(self):
        project_dir = 'scriptlet-install'
        self.run_snapcraft('build', project_dir)

        installdir = os.path.join(
            project_dir, 'parts', 'install-scriptlet-test', 'install')
        touch_file_path = os.path.join(installdir, 'build-done')
        self.assertThat(touch_file_path, FileExists())
        echoed_file_path = os.path.join(installdir, 'config.ini')
        self.assertThat(echoed_file_path, FileExists())
        self.assertThat(echoed_file_path,
                        FileContains('config-key=config-value\n'))
