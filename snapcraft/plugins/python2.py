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

"""The python2 plugin can be used for python 2 based parts.

The python2 plugin can be used for python 2 projects where you would
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

import glob
import logging
import os

import snapcraft
from snapcraft import common

logger = logging.getLogger(__name__)


class Python2Plugin(snapcraft.BasePlugin):

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
        schema.pop('required')

        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        schema['pull-properties'].extend(['requirements', 'python-packages'])

        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.stage_packages.extend([
            'python-dev',
            'python-pkg-resources',
            'python-setuptools',
        ])

    def env(self, root):
        # This is until we figure out how to get pip to download only
        # and then build in the build step or split out pulling
        # stage-packages in an internal private step.
        env = [
            'CPPFLAGS="-I{} $CPPFLAGS"'.format(os.path.join(
                root, 'usr', 'include')),
            'CFLAGS="-I{} $CFLAGS"'.format(os.path.join(
                root, 'usr', 'include')),
        ]

        # There's a chicken and egg problem here, everything run get's an
        # env built, even package installation, so the first runs for these
        # will likely fail.
        try:
            env.append('PYTHONPATH={0}'.format(common.get_python2_path(root)))
        except EnvironmentError as e:
            logger.debug(e)

        return env

    def pull(self):
        super().pull()
        self._pip()

    def _pip(self):
        setup = 'setup.py'
        if os.listdir(self.sourcedir):
            setup = os.path.join(self.sourcedir, 'setup.py')

        if self.options.requirements:
            requirements = os.path.join(os.getcwd(), self.options.requirements)

        if not os.path.exists(setup) and not \
                (self.options.requirements or self.options.python_packages):
            return

        easy_install = os.path.join(
            self.installdir, 'usr', 'bin', 'easy_install')
        prefix = os.path.join(self.installdir, 'usr')
        site_packages_dir = os.path.join(
            os.path.dirname(common.get_python2_path(self.installdir)),
            'site-packages')

        # If site-packages doesn't exist, make sure it points to the
        # dist-packages in the same directory (this is a relative link so that
        # it's still valid when the .snap is installed).
        if not os.path.exists(site_packages_dir):
            os.symlink('dist-packages', site_packages_dir)

        self.run(['python2', easy_install, '--prefix', prefix, 'pip'])

        pip2 = os.path.join(self.installdir, 'usr', 'bin', 'pip2')
        pip_install = ['python2', pip2, 'install',
                       '--global-option=build_ext',
                       '--global-option=-I{}'.format(
                           _get_python2_include(self.installdir)),
                       '--target', site_packages_dir]

        if self.options.requirements:
            self.run(pip_install + ['--requirement', requirements])

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

        os.makedirs(common.get_python2_path(self.installdir), exist_ok=True)
        self.run(
            ['python2', setup_file,
             'build_ext', '-I{}'.format(_get_python2_include(self.installdir)),
             'install', '--install-layout=deb',
             '--prefix={}/usr'.format(self.installdir),
             ], cwd=self.builddir)

    def snap_fileset(self):
        fileset = super().snap_fileset()
        fileset.append('-usr/bin/pip*')
        fileset.append('-usr/lib/python*/dist-packages/easy-install.pth')
        fileset.append('-usr/lib/python*/dist-packages/__pycache__/*.pyc')
        fileset.append('-usr/lib/python*/*.pyc')
        fileset.append('-usr/lib/python*/*/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*/*/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*/*/*/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*/*/*/*/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*/*/*/*/*/*.pyc')
        fileset.append('-usr/lib/python*/*/*/*/*/*/*/*/*/*/*.pyc')
        return fileset


def _get_python2_include(root):
    try:
        return glob.glob(os.path.join(root, 'usr', 'include', 'python2*'))[0]
    except IndexError:
        raise EnvironmentError('python development headers not installed')
