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
import datetime
import os
import shlex
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
    def _mount_snaps_directory(self) -> str:
        """Mount the host directory with snaps into the provider."""

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

    def _disable_and_wait_for_refreshes(self):
        # Disable autorefresh for 15 minutes,
        # https://github.com/snapcore/snapd/pull/5436/files
        now_plus_15 = datetime.datetime.now() + datetime.timedelta(minutes=15)
        self._run(['sudo', 'snap', 'set', 'core', 'refresh.hold={}Z'.format(
            now_plus_15.isoformat())])
        # Auto refresh may have kicked in while setting the hold.
        self.echoer.info('Waiting for pending snap auto refreshes.')
        with contextlib.suppress(errors.ProviderExecError):
            self._run(['sudo', 'snap', 'watch', '--last=auto-refresh'])

    def setup_snapcraft(self) -> None:
        self._disable_and_wait_for_refreshes()
        self.echoer.info('Setting up snapcraft in {!r}'.format(
            self.instance_name))

        # Add the store assertion, common to all snaps.
        self._inject_assertions([
            ['account-key', 'public-key-sha3-384={}'.format(
                _STORE_ASSERTION_KEY)]])

        # TODO make mounting requirement smarter and depend on is_installed
        if common.is_snap():
            # Make the snaps available to the provider
            self._mount_snaps_directory()

        # Now install the snapcraft required base/core.
        self.echoer.info('Setting up core')
        self._install_snap('core')

        # And finally install snapcraft itself.
        self.echoer.info('Setting up snapcraft')
        self._install_snap('snapcraft')

    def _inject_assertions(self, assertions: List[List[str]]):
        with tempfile.NamedTemporaryFile() as assertion_file:
            for assertion in assertions:
                assertion_file.write(repo.snaps.get_assertion(assertion))
                assertion_file.write(b'\n')
            assertion_file.flush()

            self._push_file(source=assertion_file.name,
                            destination=assertion_file.name)
            self._run(['sudo', 'snap', 'ack', assertion_file.name])

    def _install_snap(self, snap_name: str) -> None:
        snap = repo.snaps.SnapPackage(snap_name)

        args = []

        if snap.installed:
            snap_info = snap.get_local_snap_info()

            if snap_info['revision'].startswith('x'):
                args.append('--dangerous')
            else:
                self._inject_assertions([
                    ['snap-declaration', 'snap-name={}'.format(snap_name)],
                    ['snap-revision', 'snap-revision={}'.format(
                        snap_info['revision']),
                     'snap-id={}'.format(snap_info['id'])],
                ])

            if snap_info['confinement'] == 'classic':
                args.append('--classic')

            # https://github.com/snapcore/snapd/blob/master/snap/info.go
            # MountFile
            snap_file_name = '{}_{}.snap'.format(
                snap_name, snap_info['revision'])
            args.append(os.path.join(self._SNAPS_MOUNTPOINT, snap_file_name))
        else:
            snap_info = snap.get_store_snap_info()
            # TODO support other channels
            confinement = snap_info['channels']['latest/stable']['confinement']
            if confinement == 'classic':
                args.append('--classic')
            args.append(snap_name)

        self._run(['sudo', 'snap', 'install'] + args)
