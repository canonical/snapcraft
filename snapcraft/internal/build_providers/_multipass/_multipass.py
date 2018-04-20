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
from contextlib import contextmanager
from typing import Generator

from .. import errors
from .._base_provider import BaseProvider
from ._instance_info import InstanceInfo
from ._multipass_command import MultipassCommand


class Multipass(BaseProvider):
    """A multipass provider for snapcraft execute its lifecycle."""

    def __init__(self, *, project, echoer) -> None:
        self._multipass_cmd = MultipassCommand()
        super().__init__(project=project, echoer=echoer,
                         executor=self._multipass_cmd.execute)

    @contextmanager
    def new_instance(self, keep=True) -> Generator:
        """Create the multipass instance."""
        try:
            self.launch_instance(self._multipass_cmd.launch, image='16.04')
            self.setup_snapcraft()
            yield self
        finally:
            try:
                self._try_stop_and_delete(keep=keep)
            except errors.ProviderStopError as stop_error:
                self.echoer.warning('Could not stop {!r}: {}.'.format(
                    self.instance_name, stop_error))
            except errors.ProviderDeleteError as delete_error:
                self.echoer.warning('Could not stop {!r}: {}.'.format(
                    self.instance_name, delete_error))

    def _try_stop_and_delete(self, *, keep: bool) -> None:
        try:
            instance_info = self._get_instance_info()
        except errors.ProviderInfoError as info_error:
            # An error here means this instance may not exist
            return

        if instance_info.is_stopped():
            return
        self._multipass_cmd.stop(instance_name=self.instance_name)
        if not keep:
            self._multipass_cmd.delete(instance_name=self.instance_name)

    def provision_project(self, tarball: str) -> None:
        """Provision the multipass instance with the project to work with."""
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
        # Use the full path as /snap/bin is not in PATH.
        snapcraft_cmd = 'cd {}; /snap/bin/snapcraft snap --output {}'.format(
            self.project_dir, self.snap_filename)
        self._multipass_cmd.execute(command=['sh', '-c', snapcraft_cmd],
                                    instance_name=self.instance_name)

    def retrieve_snap(self) -> str:
        source = '{}:{}/{}'.format(self.instance_name,
                                   self.project_dir,
                                   self.snap_filename)
        self._multipass_cmd.copy_files(source=source,
                                       destination=self.snap_filename)
        return self.snap_filename

    def _get_instance_info(self):
        instance_info_raw = self._multipass_cmd.info(
            instance_name=self.instance_name, output_format='json')
        return InstanceInfo.new_instance_info_from_json(
            instance_name=self.instance_name,
            json_info=instance_info_raw.decode())
