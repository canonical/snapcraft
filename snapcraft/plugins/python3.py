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

"""The python3 plugin can be used for python 3 based parts.

The python3 plugin can be used for python 3 projects where you would
want to do:

    - import python modules with a requirements.txt
    - build a python project that has a setup.py
    - install sources straight from pip

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - requirements:
      (string)
      path to a requirements.txt file
    - python-packages:
      (list)
      A list of dependencies to get from PyPi
"""

import os
import re

import snapcraft
from snapcraft import common


class Python3Plugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['requirements'] = {
            'type': 'string',
        }
        schema['properties']['python-packages'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string'
            },
            'default': [],
        }
        schema['properties']['process-dependency-links'] = {
            'type': 'boolean',
            'default': False,
        }
        schema.pop('required')

        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        schema['pull-properties'].extend(['requirements', 'python-packages'])

        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.stage_packages.extend([
            'python3-dev',
            'python3-pkg-resources',
            'python3-setuptools',
        ])

    def env(self, root):
        return [
            'PYTHONPATH={}'.format(os.path.join(
                root, 'usr', 'lib', self.python_version, 'site-packages')),
            # This is until we figure out how to get pip to download only
            # and then build in the build step or split out pulling
            # stage-packages in an internal private step.
            'CPPFLAGS="-I{} $CPPFLAGS"'.format(os.path.join(
                root, 'usr', 'include')),
            'CFLAGS="-I{} $CFLAGS"'.format(os.path.join(
                root, 'usr', 'include')),
        ]

    def pull(self):
        super().pull()

        setup = 'setup.py'
        if os.listdir(self.sourcedir):
            setup = os.path.join(self.sourcedir, 'setup.py')

        if (os.path.exists(setup) or self.options.requirements or
                self.options.python_packages):
            self._pip(setup)

    def _setup_pip(self):
        easy_install = os.path.join(
            self.installdir, 'usr', 'bin', 'easy_install3')
        prefix = os.path.join(self.installdir, 'usr')

        site_packages_dir = os.path.join(
            prefix, 'lib', self.python_version, 'site-packages')
        # If site-packages doesn't exist, make sure it points to the
        # python3 dist-packages (this is a relative link so that it's still
        # valid when the .snap is installed). Note that all python3 versions
        # share the same dist-packages (e.g. in python3, not python3.4).
        if not os.path.exists(site_packages_dir):
            os.symlink(os.path.join('..', 'python3', 'dist-packages'),
                       site_packages_dir)

        self.run(['python3', easy_install, '--prefix', prefix, 'pip'])

    def _get_pip_command(self):
        pip3 = os.path.join(self.installdir, 'usr', 'bin', 'pip3')
        pip_install = ['python3', pip3, 'install', '--root',
                       self.installdir, "--install-option=--prefix=usr"]

        if self.options.process_dependency_links:
            pip_install.append('--process-dependency-links')

        return pip_install

    def _pip(self, setup):
        self._setup_pip()
        pip_install = self._get_pip_command()

        if self.options.requirements:
            self.run(pip_install + [
                '--requirement',
                os.path.join(self.sourcedir, self.options.requirements),
            ])

        if self.options.python_packages:
            self.run(pip_install + ['--upgrade'] +
                     self.options.python_packages)
        if os.path.exists(setup):
            self.run(pip_install + ['.', ], cwd=self.sourcedir)

    def build(self):
        super().build()

        # If setuptools is used, it tries to create files in the
        # dist-packages dir and import from there, so it needs to exist
        # and be in the PYTHONPATH. It's harmless if setuptools isn't
        # used.

        setup_file = os.path.join(self.builddir, 'setup.py')
        if not os.path.exists(setup_file):
            return

        os.makedirs(self.dist_packages_dir, exist_ok=True)
        self.run(
            ['python3', setup_file, 'install', '--install-layout=deb',
             '--prefix={}/usr'.format(self.installdir),
             '--root={}'.format(self.installdir)], cwd=self.builddir)

        # Fix all shebangs to use the in-snap python.
        common.replace_in_file(self.installdir, re.compile(r''),
                               re.compile(r'#!.*python'),
                               r'#!/usr/bin/env python')

    @property
    def dist_packages_dir(self):
        return os.path.join(
            self.installdir, 'usr', 'lib', self.python_version,
            'dist-packages')

    @property
    def python_version(self):
        return self.run_output(['py3versions', '-d'])

    def snap_fileset(self):
        fileset = super().snap_fileset()
        fileset.append('-usr/bin/pip*')
        fileset.append('-usr/lib/python*/__pycache__/*.pyc')
        fileset.append('-usr/lib/python*/*/__pycache__/*.pyc')
        fileset.append('-usr/lib/python*/*/*/__pycache__/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/__pycache__/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*/__pycache__/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*/*/__pycache__/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*/*/*/__pycache__/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*/*/*/*/__pycache__/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*/*/*/*/*/__pycache__/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*/*/*/*/*/*/__pycache__/*.pyc')
        return fileset
