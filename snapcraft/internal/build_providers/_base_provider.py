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
import contextlib
import os
import shlex
import subprocess
import tempfile
from typing import List

import petname

from . import errors
from snapcraft.internal import common, repo


_STORE_ASSERTION_KEY = (
    'BWDEoaqyr25nF5SNCvEv2v7QnM9QsfCc0PBMYD_i2NGSQ32EF2d4D0hqUel3m8ul')


class Provider():

    _SNAPS_MOUNTPOINT = os.path.join(os.path.sep, 'var', 'cache', 'snapcraft',
                                     'snaps')

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

    @abc.abstractproperty
    def _snaps_path_or_dev(self) -> str:
        """Return the path or device exposed in the provider to mount."""

    @abc.abstractmethod
    def _run(self, command: List) -> None:
        """Run a command on the instance."""

    @abc.abstractmethod
    def _launch(self):
        """Launch the instance."""

    @abc.abstractmethod
    def _mount(self, *, mountpoint: str, dev_or_path: str) -> None:
        """Mount a path from the host inside the instance."""

    @abc.abstractmethod
    def _push_file(self, *, source: str, destination: str) -> None:
        """Push a file into the instance."""

    @abc.abstractmethod
    def create(self) -> None:
        """Provider steps needed to create a fully functioning environment."""

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
    def mount_project(self) -> None:
        """Provider steps needed to make the project available to the instance.
        """

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
        if common.is_snap():
            self._inject_snapcraft()
        else:
            self._install_snapcraft()

    def _install_snapcraft(self) -> None:
        """Install snapcraft from the store."""
        install_cmd = ['sudo', 'snap', 'install', 'snapcraft', '--classic']
        self._run(install_cmd)

    def _inject_snapcraft(self) -> None:
        """Install snapcraft using local assets."""
        # Make the snaps available to the provider
        self._mount(mountpoint=self._SNAPS_MOUNTPOINT,
                    dev_or_path=self._snaps_path_or_dev)
        # TODO this should be properly fixed in snapd.
        self.echoer.info('Waiting for pending snap auto refreshes.')
        with contextlib.suppress(errors.ProviderExecError):
            self._run(['sudo', 'snap', 'watch', '--last=auto-refresh'])
        # Add the store assertion, common to all snaps.
        self._inject_assertions([
            ['account-key', 'public-key-sha3-384={}'.format(
                _STORE_ASSERTION_KEY)]])
        self.echoer.info('Setting up core')
        self._install_snap('core')
        self.echoer.info('Setting up snapcraft')
        self._install_snap('snapcraft')

    def _inject_assertions(self, assertions: List[List[str]]):
        with tempfile.NamedTemporaryFile() as assertion_file:
            for assertion in assertions:
                assertion_file.write(
                    subprocess.check_output(['snap', 'known', *assertion]))
                assertion_file.write(b'\n')
            assertion_file.flush()

            self._push_file(source=assertion_file.name,
                            destination=assertion_file.name)
            self._run(['sudo', 'snap', 'ack', assertion_file.name])

    def _inject_snap(self, snap_file_path: str, *, is_dangerous: bool,
                     is_classic: bool) -> None:
        # Install: will do nothing if already installed
        args = []
        if is_dangerous:
            args.append('--dangerous')
        if is_classic:
            args.append('--classic')

        self._run(['sudo', 'snap', 'install', snap_file_path] + args)

    def _install_snap(self, snap_name: str) -> None:
        snap = repo.snaps.SnapPackage(snap_name)
        if not snap.installed:
            raise repo.errors.SnapFindError(snap_name=snap_name)
        snap_info = snap.get_local_snap_info()
        snap_id = snap_info['id']
        snap_rev = snap_info['revision']
        is_classic = snap_info['confinement'] == 'classic'
        is_dangerous = snap_rev.startswith('x')

        if not is_dangerous:
            self._inject_assertions([
                ['snap-declaration', 'snap-name={}'.format(snap_name)],
                ['snap-revision', 'snap-revision={}'.format(snap_rev),
                 'snap-id={}'.format(snap_id)],
            ])

        # https://github.com/snapcore/snapd/blob/master/snap/info.go
        # MountFile
        snap_file_name = '{}_{}.snap'.format(snap_name, snap_rev)
        snap_file_path = os.path.join(self._SNAPS_MOUNTPOINT, snap_file_name)
        self._inject_snap(snap_file_path,
                          is_classic=is_classic,
                          is_dangerous=is_dangerous)
