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

import os
import glob
import platform
import integration_tests
from contextlib import contextmanager


@contextmanager
def cd(path):
    old_dir = os.getcwd()
    os.chdir(path)
    yield
    os.chdir(old_dir)


class RubyPluginTestCase(integration_tests.TestCase):

    def env(self, root, ruby_version_dir):
        env = []
        env.append("PATH={}:{}".format(
            os.path.join(root, "bin"), os.environ["PATH"]))

        env.append("RUBYPATH={}".format(os.path.join(root, "bin")))

        rubydir = os.path.join(root, "lib", "ruby")


        rubylib = os.path.join(rubydir, ruby_version_dir)

        env.append("RUBYLIB={}:{}".format(
            rubylib, os.path.join(
                rubylib, "{}-linux".format(platform.machine()))))

        env.append("GEM_HOME={}".format(
            os.path.join(rubydir, "gems", ruby_version_dir)))

        env.append("GEM_PATH={}".format(
            os.path.join(rubydir, "gems", ruby_version_dir)))

        return env

    def test_bins_exist(self):
        self.run_snapcraft('stage', 'ruby-bins-exist')
        for exe in ['erb', 'gem', 'irb', 'rake', 'rdoc', 'ri', 'ruby']:
            exe_path = os.path.join(self.stage_dir, 'bin', exe)
            self.assertTrue(os.path.exists(exe_path))

    def test_ruby_gem_install_rack(self):
        self.run_snapcraft('stage', 'ruby-gem-install-rack')
        rack_path = os.path.join(self.stage_dir, 'bin', 'rack')
        self.assertTrue(os.path.exists(rack_path))

    def test_ruby_hello(self):
        self.run_snapcraft('stage', 'ruby-hello')
        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'bin', os.path.basename(self.path)))
        self.assertEqual("Ruby says, Hello snapcraft.\n", binary_output)
