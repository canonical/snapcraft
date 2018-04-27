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

import abc
import shlex
from typing import List

import petname


class Provider():

    def __init__(self, *, project, echoer) -> None:
        self.project = project
        self.echoer = echoer
        # Once https://github.com/CanonicalLtd/multipass/issues/220 is
        # closed we can prepend snapcraft- again.
        self.instance_name = petname.Generate(2, '-')
        self.project_dir = shlex.quote(project.info.name)

        if project.info.version:
            self.snap_filename = '{}_{}_{}.snap'.format(
                project.info.name, project.info.version, project.deb_arch)
        else:
            self.snap_filename = '{}_{}.snap'.format(
                project.info.name, project.deb_arch)

    def __enter__(self):
        self.create()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.destroy()

    @abc.abstractmethod
    def _run(self, command: List) -> None:
        """Run a command on the instance."""

    @abc.abstractmethod
    def _launch(self):
        """Launch the instance."""

    @abc.abstractmethod
    def create(self) -> None:
        """Provider steps needed to create a fully functioning environemnt."""

    @abc.abstractmethod
    def destroy(self) -> None:
        """Provider steps needed to ensure the instance is destroyed.

        This method should be safe to call multiple times and do nothing
        if the instance to destroy is already destroyed.
        """

    @abc.abstractmethod
    def provision_project(self, tarball: str) -> None:
        """Provider steps needed to copy project assests to the instance."""

    @abc.abstractmethod
    def build_project(self) -> None:
        """Provider steps needed build the project on the instance."""

    @abc.abstractmethod
    def retrieve_snap(self) -> str:
        """
        Provider steps needed to retrieve the built snap from the instance.

        :returns: the filename of the retrieved snap.
        :rtype: str
        """

    def launch_instance(self) -> None:
        self.echoer.info('Creating a build environment named {!r}'.format(
            self.instance_name))
        self._launch()

    def setup_snapcraft(self) -> None:
        self.echoer.info('Setting up snapcraft in {!r}'.format(
            self.instance_name))
        install_cmd = ['sudo', 'snap', 'install', 'snapcraft', '--classic']
        self._run(install_cmd)
