# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright © 2015 Canonical Ltd
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
import tempfile

import snapcraft


class Python2ProjectPlugin(snapcraft.BasePlugin):

    # note that we don't need to setup env(), python figures it out
    # see python2.py for more details

    def pull(self):
        return self.handle_source_options()

    def build(self):
        # If setuptools is used, it tries to create files in the
        # dist-packages dir and import from there, so it needs to exist
        # and be in the PYTHONPATH. It's harmless if setuptools isn't
        # used.
        os.makedirs(self.dist_packages_dir, exist_ok=True)
        setuptemp = self.copy_setup()

        return self.run(
            ['python2', setuptemp.name, 'install', '--install-layout=deb',
             '--prefix={}/usr'.format(self.installdir)], cwd=self.builddir)

    @property
    def dist_packages_dir(self):
        return os.path.join(
            self.installdir, 'usr', 'lib', 'python2.7', 'dist-packages')

    # Takes the setup.py file and puts a couple little gems on the
    # front to make things work better.
    def copy_setup(self):
        setupout = tempfile.NamedTemporaryFile(dir=self.builddir, mode='w+')

        setupout.write('import sys\n')
        setupout.write('sys.executable = "usr/bin/python2"\n\n')

        with open(os.path.join(self.builddir, 'setup.py'), 'r') as f:
            for line in f:
                setupout.write(line)

        setupout.flush()
        return setupout
