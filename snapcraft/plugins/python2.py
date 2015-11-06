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

"""The python2 plugin can be used for python 2 based parts.

The python2 plugin can be used for python 2 projects where you would
want to do:

    - import python modules with a requirements.txt
    - build a python project that has a setup.py
    - install sources straight from pip

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin specific keywords:

    - requirements:
      (string)
      path to a requirements.txt file
"""

import os
import tempfile

import snapcraft


class Python2Plugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['requirements'] = {
            'type': 'string',
        }
        schema.pop('required')

        return schema

    def __init__(self, name, options):
        super().__init__(name, options)
        self.stage_packages.extend([
            'python-dev',
            'python-pkg-resources',
            'python-setuptools',
        ])

    def env(self, root):
        return ["PYTHONPATH={}".format(os.path.join(
            root, 'usr', 'lib', self.python_version, 'dist-packages'))]

    def pull(self):
        super().pull()
        self._pip()

    def _pip(self):
        setup = 'setup.py'
        if os.listdir(self.sourcedir):
            setup = os.path.join(self.sourcedir, 'setup.py')

        if self.options.requirements:
            requirements = os.path.join(os.getcwd(), self.options.requirements)

        if not os.path.exists(setup) and not self.options.requirements:
            return

        easy_install = os.path.join(
            self.installdir, 'usr', 'bin', 'easy_install')
        prefix = os.path.join(self.installdir, 'usr')
        site_packages_dir = os.path.join(
            prefix, 'lib', self.python_version, 'site-packages')

        if not os.path.exists(site_packages_dir):
            os.symlink(
                os.path.join(prefix, 'lib', self.python_version,
                             'dist-packages'),
                site_packages_dir)

        self.run(['python2', easy_install, '--prefix', prefix, 'pip'])

        pip2 = os.path.join(self.installdir, 'usr', 'bin', 'pip2')
        pip_install = ['python2', pip2, 'install', '--target',
                       site_packages_dir]

        if self.options.requirements:
            self.run(pip_install + ['--requirement', requirements])

        if os.path.exists(setup):
            self.run(pip_install + ['.', ], cwd=self.sourcedir)

    def build(self):
        super().build()
        # If setuptools is used, it tries to create files in the
        # dist-packages dir and import from there, so it needs to exist
        # and be in the PYTHONPATH. It's harmless if setuptools isn't
        # used.

        if not os.path.exists(os.path.join(self.builddir, 'setup.py')):
            return

        os.makedirs(self.dist_packages_dir, exist_ok=True)
        setuptemp = self.copy_setup()
        self.run(
            ['python2', setuptemp.name, 'install', '--install-layout=deb',
             '--prefix={}/usr'.format(self.installdir)], cwd=self.builddir)

    @property
    def dist_packages_dir(self):
        return os.path.join(
            self.installdir, 'usr', 'lib', self.python_version,
            'dist-packages')

    @property
    def python_version(self):
        return self.run_output(['pyversions', '-i'])

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
