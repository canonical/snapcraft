# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 James Beedy <jamesbeedy@gmail.com>
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

from contextlib import contextmanager
import glob
import os

import integration_tests


@contextmanager
def cd(path):
    old_dir = os.getcwd()
    os.chdir(path)
    yield
    os.chdir(old_dir)


class RubyPluginTestCase(integration_tests.TestCase):

    def test_bins_exist(self):
        self.run_snapcraft('stage', 'ruby-bins-exist')
        for exe in ['erb', 'gem', 'irb', 'rake', 'rdoc', 'ri', 'ruby']:
            exe_path = os.path.join(self.stage_dir, 'bin', exe)
            self.assertTrue(os.path.exists(exe_path))

    def test_ruby_gem_install_rack(self):
        self.run_snapcraft('stage', 'ruby-gem-install-rack')
        rack_path = os.path.join(self.stage_dir, 'bin', 'rackup')
        self.assertTrue(os.path.exists(rack_path))
