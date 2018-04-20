# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import shlex

import petname


# This class is a mix of a parent and a mixin.
class BaseProvider:

    def __init__(self, *, project, executor, echoer) -> None:
        self.project = project
        self.echoer = echoer
        self.instance_name = petname.Generate(2, '-')
        self.project_dir = shlex.quote(project.info.name)
        self._exec = executor

        if project.info.version:
            self.snap_filename = '{}_{}_{}.snap'.format(
                project.info.name, project.info.version, project.deb_arch)
        else:
            self.snap_filename = '{}_{}.snap'.format(
                project.info.name, project.deb_arch)

    def launch_instance(self, method, **kwargs):
        self.echoer.info('Creating a build environment named {!r}'.format(
            self.instance_name))
        method(instance_name=self.instance_name, **kwargs)

    def setup_snapcraft(self):
        self.echoer.info('Setting up snapcraft in {!r}'.format(
            self.instance_name))
        install_cmd = ['sudo', 'snap', 'install', 'snapcraft', '--classic']
        self._exec(command=install_cmd, instance_name=self.instance_name)
