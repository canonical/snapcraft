# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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


class Python3ProjectPlugin(snapcraft.BasePlugin):

    _PLUGIN_STAGE_PACKAGES = [
        'python3-dev',
        'python3-pkg-resources',
        'python3-setuptools',
    ]

    def __init__(self, name, options):
        super().__init__(name, options)
        self.requirements = options.requirements
        self.source = options.source

    def env(self, root):
        return ["PYTHONPATH=%s" % os.path.join(
            root, 'usr', 'lib', self.python_version, 'dist-packages')]

    def pull(self):
        if self.source and not self.handle_source_options():
            return False

        return self._pip()

    def _pip(self):
        setup = 'setup.py'
        if os.listdir(self.sourcedir):
            setup = os.path.join(self.sourcedir, 'setup.py')

        if self.requirements:
            requirements = os.path.join(os.getcwd(), self.requirements)

        if not os.path.exists(setup) and not self.requirements:
            return True

        easy_install = os.path.join(
            self.installdir, 'usr', 'bin', 'easy_install3')
        prefix = os.path.join(self.installdir, 'usr')
        site_packages_dir = os.path.join(
            prefix, 'lib', self.python_version, 'site-packages')

        if not os.path.exists(site_packages_dir):
            os.symlink(
                os.path.join(prefix, 'lib', 'python3', 'dist-packages'),
                site_packages_dir)

        if not self.run(['python3', easy_install, '--prefix', prefix, 'pip']):
            return False

        pip3 = os.path.join(self.installdir, 'usr', 'bin', 'pip3')
        pip_install = ['python3', pip3, 'install', '--target',
                       site_packages_dir]

        if self.requirements and not self.run(
                pip_install + ['--requirement', requirements]):
            return False

        if os.path.exists(setup) and not self.run(pip_install + ['.', ]):
            return False

        return True

    def build(self):
        # If setuptools is used, it tries to create files in the
        # dist-packages dir and import from there, so it needs to exist
        # and be in the PYTHONPATH. It's harmless if setuptools isn't
        # used.

        if not os.path.exists(os.path.join(self.builddir, 'setup.py')):
            return True

        os.makedirs(self.dist_packages_dir, exist_ok=True)
        setuptemp = self.copy_setup()
        return self.run(
            ['python3', setuptemp.name, 'install', '--install-layout=deb',
             '--prefix={}/usr'.format(self.installdir)], cwd=self.builddir)

    @property
    def dist_packages_dir(self):
        return os.path.join(
            self.installdir, 'usr', 'lib', self.python_version, 'dist-packages')

    @property
    def python_version(self):
        return self.run_output(['py3versions', '-i'])

    # Takes the setup.py file and puts a couple little gems on the
    # front to make things work better.
    def copy_setup(self):
        setupout = tempfile.NamedTemporaryFile(dir=self.builddir, mode='w+')

        setupout.write('import sys\n')
        setupout.write('sys.executable = "usr/bin/python3"\n\n')

        with open(os.path.join(self.builddir, 'setup.py'), 'r') as f:
            for line in f:
                setupout.write(line)

        setupout.flush()
        return setupout
