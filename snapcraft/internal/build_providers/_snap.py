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

import contextlib
import datetime
import enum
import logging
import os
import tempfile
from typing import List
from typing import Any, Dict, Union  # noqa: F401

import yaml

from . import errors
from snapcraft.internal import repo

logger = logging.getLogger(__name__)


class _SnapOp(enum.Enum):
    NOP = 0
    INJECT = 1
    INSTALL = 2
    REFRESH = 3


class _SnapManager:
    def __init__(self, *, snap_name: str, snap_dir: str, latest_revision: str) -> None:
        self.snap_name = snap_name
        self._snap_dir = snap_dir

        self._latest_revision = latest_revision
        self.__required_operation = None  # type: Union[None, _SnapOp]
        self.__repo = None  # type: Union[None, repo.snaps.SnapPackage]
        self.__revision = None  # type: Union[None, str]
        self.__install_cmd = None  # type: Union[None, List[str]]

    def _get_snap_repo(self):
        if self.__repo is None:
            self.__repo = repo.snaps.SnapPackage(self.snap_name)
        return self.__repo

    def get_op(self) -> _SnapOp:
        if self.__required_operation is not None:
            return self.__required_operation

        # Get information from the host.
        host_snap_repo = self._get_snap_repo()
        host_snap_info = host_snap_repo.get_local_snap_info()

        # The evaluations for the required operation is as follows:
        # - if the snap is not installed on the host (host_snap_repo.installed == False),
        #   and the snap is not installed in the build environment
        #   (_latest_revision is None), then a store install will take place.
        # - else if the snap is not installed on the host (host_snap_repo.installed == False),
        #   but the is previously installed revision in the build environment
        #   (_latest_revision is not None), then a store install will take place.
        # - else if the snap is installed on the host (host_snap_repo.installed == True),
        #   and the snap installed in the build environment (_latest_revision) matches
        #   the one on the host, no operation takes place.
        # - else if the snap is installed on the host (host_snap_repo.installed == True),
        #   and the snap installed in the build environment (_latest_revision) does not
        #   match the one on the host, then a snap injection from the host will take place.
        if not host_snap_repo.installed and self._latest_revision is None:
            op = _SnapOp.INSTALL
        elif not host_snap_repo.installed and self._latest_revision is not None:
            op = _SnapOp.REFRESH
        elif (
            host_snap_repo.installed
            and self._latest_revision == host_snap_info["revision"]
        ):
            op = _SnapOp.NOP
        elif (
            host_snap_repo.installed
            and self._latest_revision != host_snap_info["revision"]
        ):
            op = _SnapOp.INJECT
        else:
            # This is a programmatic error
            raise RuntimeError(
                "Unhandled scenario for {!r} (host installed: {}, latest_revision {})".format(
                    self.snap_name, host_snap_repo.installed, self._latest_revision
                )
            )

        self.__required_operation = op
        return op

    def get_assertions(self) -> List[List[str]]:
        op = self.get_op()
        if op != op.INJECT:
            return []
        host_snap_repo = self._get_snap_repo()
        host_snap_info = host_snap_repo.get_local_snap_info()

        if host_snap_info["revision"].startswith("x"):
            return []

        assertions = []  # type: List[List[str]]
        assertions.append(
            ["snap-declaration", "snap-name={}".format(host_snap_repo.name)]
        )
        assertions.append(
            [
                "snap-revision",
                "snap-revision={}".format(host_snap_info["revision"]),
                "snap-id={}".format(host_snap_info["id"]),
            ]
        )
        return assertions

    def _set_data(self) -> None:
        op = self.get_op()
        host_snap_repo = self._get_snap_repo()
        install_cmd = ["sudo", "snap"]

        if op == _SnapOp.INJECT:
            install_cmd.append("install")
            host_snap_info = host_snap_repo.get_local_snap_info()
            snap_revision = host_snap_info["revision"]

            if snap_revision.startswith("x"):
                install_cmd.append("--dangerous")

            if host_snap_info["confinement"] == "classic":
                install_cmd.append("--classic")

            snap_file_name = "{}_{}.snap".format(host_snap_repo.name, snap_revision)
            install_cmd.append(os.path.join(self._snap_dir, snap_file_name))
        elif op == _SnapOp.INSTALL or op == _SnapOp.REFRESH:
            install_cmd.append(op.name.lower())
            host_snap_info = host_snap_repo.get_store_snap_info()
            snap_revision = host_snap_info["channels"]["latest/stable"]["revision"]
            confinement = host_snap_info["channels"]["latest/stable"]["confinement"]
            if confinement == "classic":
                install_cmd.append("--classic")
            install_cmd.append(host_snap_repo.name)
        elif op == _SnapOp.NOP:
            install_cmd = []
            snap_revision = None

        self.__install_cmd = install_cmd
        self.__revision = snap_revision

    def get_revision(self) -> str:
        if self.__revision is None:
            self._set_data()
        return self.__revision

    def get_install_cmd(self) -> List[str]:
        if self.__install_cmd is None:
            self._set_data()
        return self.__install_cmd


_STORE_ASSERTION = [
    "account-key",
    "public-key-sha3-384=BWDEoaqyr25nF5SNCvEv2v7QnM9QsfCc0PBMYD_i2NGSQ32EF2d4D0hqUel3m8ul",
]


class SnapInjector:
    """Handle the process of adding snaps into the build environment.

    The specific knowledge of the build environment where these snaps will
    be injected is not required, instead runnables to execute the required
    operations against the build environment are provided upon initialization.

    The snaps to install or refresh in the build environment are added by calling
    add and finally applied by calling apply. If an operation is required, the
    revision that eventually made it into the environment is recorded in the
    registry.
    """

    def __init__(
        self,
        *,
        snap_dir: str,
        registry_filepath: str,
        runner,
        snap_dir_mounter,
        snap_dir_unmounter,
        file_pusher
    ) -> None:
        """
        Initialize a SnapInjector instance.

        :param str snap_dir: directory where snaps from the host live for the cases
                             where injection of the snap into the build environment
                             is possible.
        :param str registry_filepath: path to where recordings of previusly installed
                                      revisions of a snap can be queried and recorded.
        :param runner: a callable which can run commands in the build environment.
        :param snap_dir_mounter: a callable which can mount the directory where snaps live
                                 on the host into the build environment. This callable
                                 must mount into snap_dir.
        :param snap_dir_unmounter: a callable which can unmount snap_dir from the environment.
        :param file_pusher: a callable that can push file from the host into the build
                            environment.
        """

        self._snaps = []  # type: List["_SnapManager"]
        self._snap_dir = snap_dir
        self._registry_filepath = registry_filepath
        self._runner = runner
        self._snap_dir_mounter = snap_dir_mounter
        self._snap_dir_unmounter = snap_dir_unmounter
        self._file_pusher = file_pusher

        self._registry_data = dict()  # type: Union[None, Dict[str, List[Any]]]

    def _load_registry(self):
        if self._registry_filepath is None or self._registry_data:
            return
        if not os.path.exists(self._registry_filepath):
            return

        with open(self._registry_filepath) as registry_file:
            self._registry_data = yaml.load(registry_file)

    def _dump_registry(self):
        if self._registry_filepath is None or self._registry_data is None:
            return

        dirpath = os.path.dirname(self._registry_filepath)
        if dirpath:
            os.makedirs(dirpath, exist_ok=True)

        with open(self._registry_filepath, "w") as registry_file:
            yaml.dump(self._registry_data, stream=registry_file)

    @contextlib.contextmanager
    def _mounted_dir(self):
        if any((s.get_op() == _SnapOp.INJECT for s in self._snaps)):
            self._snap_dir_mounter()
            try:
                yield
            finally:
                self._snap_dir_unmounter()
        else:
            yield

    def _disable_and_wait_for_refreshes(self):
        # Disable autorefresh for 15 minutes,
        # https://github.com/snapcore/snapd/pull/5436/files
        now_plus_15 = datetime.datetime.now() + datetime.timedelta(minutes=15)
        self._runner(
            [
                "sudo",
                "snap",
                "set",
                "core",
                "refresh.hold={}Z".format(now_plus_15.isoformat()),
            ]
        )
        # Auto refresh may have kicked in while setting the hold.
        logger.debug("Waiting for pending snap auto refreshes.")
        with contextlib.suppress(errors.ProviderExecError):
            self._runner(
                ["sudo", "snap", "watch", "--last=auto-refresh"], hide_output=True
            )

    def _inject_assertions(self) -> None:
        assertions = []  # type: List[List[str]]
        for snap in self._snaps:
            assertions.extend(snap.get_assertions())
        if assertions:
            assertions = [_STORE_ASSERTION] + assertions

        # Return early if not assertions need acking
        if not assertions:
            return

        with tempfile.NamedTemporaryFile() as assertion_file:
            for assertion in assertions:
                assertion_file.write(repo.snaps.get_assertion(assertion))
                assertion_file.write(b"\n")
            assertion_file.flush()

            self._file_pusher(
                source=assertion_file.name, destination=assertion_file.name
            )
            self._runner(["sudo", "snap", "ack", assertion_file.name])

    def _get_latest_revision(self, snap_name):
        self._load_registry()
        try:
            return self._registry_data[snap_name][-1]["revision"]
        except (IndexError, KeyError):
            return None

    def _add_latest_revision(self, snap_name: str, snap_revision: str) -> None:
        entry = dict(revision=snap_revision)

        if snap_name not in self._registry_data:
            self._registry_data[snap_name] = [entry]
        else:
            self._registry_data[snap_name].append(entry)

    def add(self, snap_name: str) -> None:
        self._snaps.append(
            _SnapManager(
                snap_name=snap_name,
                snap_dir=self._snap_dir,
                latest_revision=self._get_latest_revision(snap_name),
            )
        )

    def apply(self):
        if all((s.get_op() == _SnapOp.NOP for s in self._snaps)):
            return

        # Disable refreshes so they do not interfere with installation ops.
        self._disable_and_wait_for_refreshes()

        # Before injecting any snap we must make sure the required assertions
        # are acked.
        self._inject_assertions()

        with self._mounted_dir():
            for snap in self._snaps:
                install_cmd = snap.get_install_cmd()
                self._runner(install_cmd)
                self._add_latest_revision(snap.snap_name, snap.get_revision())

        self._dump_registry()
