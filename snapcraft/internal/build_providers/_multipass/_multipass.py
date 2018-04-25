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

from .. import errors
from .._base_provider import BaseProvider
from ._instance_info import InstanceInfo
from ._multipass_command import MultipassCommand


class Multipass(BaseProvider):
    """A multipass provider for snapcraft to execute its lifecycle."""

    def __init__(self, *, project, echoer) -> None:
        self._multipass_cmd = MultipassCommand()
        super().__init__(project=project, echoer=echoer,
                         executor=self._multipass_cmd.execute)

    def __enter__(self):
        self.create()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.destroy()

    def create(self):
        """Create the multipass instance and setup the build environment."""
        self.launch_instance(self._multipass_cmd.launch, image='16.04')
        self.setup_snapcraft()

    def destroy(self):
        """Destroy the instance, trying to stop it first."""
        try:
            instance_info = self._get_instance_info()
        except errors.ProviderInfoError as info_error:
            self.echoer.warning(
                'Failed to obtain the status of {!r} when trying to '
                'delete: {}'.format(self.instance_name, info_error))
            return

        try:
            if not instance_info.is_stopped():
                self._multipass_cmd.stop(instance_name=self.instance_name)
            self._multipass_cmd.delete(instance_name=self.instance_name)
        except errors.ProviderStopError as stop_error:
            self.echoer.warning('Could not stop {!r}: {}.'.format(
                self.instance_name, stop_error))
        except errors.ProviderDeleteError as stop_error:
            self.echoer.warning('Could not stop {!r}: {}.'.format(
                self.instance_name, stop_error))

    def provision_project(self, tarball: str) -> None:
        """Provision the multipass instance with the project to work with."""
        # TODO add instance check.
        # Step 0, sanitize the input
        tarball = shlex.quote(tarball)

        # First create a working directory
        self._multipass_cmd.execute(command=['mkdir', self.project_dir],
                                    instance_name=self.instance_name)

        # Then copy the tarball over
        destination = '{}:{}'.format(self.instance_name, tarball)
        self._multipass_cmd.copy_files(source=tarball, destination=destination)

        # Finally extract it into project_dir.
        extract_cmd = ['tar', '-xvf', tarball, '-C', self.project_dir]
        self._multipass_cmd.execute(command=extract_cmd,
                                    instance_name=self.instance_name)

    def build_project(self) -> None:
        # TODO add instance check.
        # Use the full path as /snap/bin is not in PATH.
        snapcraft_cmd = 'cd {}; /snap/bin/snapcraft snap --output {}'.format(
            self.project_dir, self.snap_filename)
        self._multipass_cmd.execute(command=['sh', '-c', snapcraft_cmd],
                                    instance_name=self.instance_name)

    def retrieve_snap(self) -> str:
        # TODO add instance check.
        source = '{}:{}/{}'.format(self.instance_name,
                                   self.project_dir,
                                   self.snap_filename)
        self._multipass_cmd.copy_files(source=source,
                                       destination=self.snap_filename)
        return self.snap_filename

    def _get_instance_info(self):
        instance_info_raw = self._multipass_cmd.info(
            instance_name=self.instance_name, output_format='json')
        return InstanceInfo.from_json(instance_name=self.instance_name,
                                      json_info=instance_info_raw.decode())
