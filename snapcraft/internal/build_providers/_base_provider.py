# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2020 Canonical Ltd
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
import os
import pathlib
import logging
import shlex
import shutil
import sys
from textwrap import dedent
from typing import Optional, Sequence
from typing import Any, Dict

from xdg import BaseDirectory

from . import errors
from ._snap import SnapInjector
from snapcraft.internal import common, steps
from snapcraft import yaml_utils


logger = logging.getLogger(__name__)


def _get_platform() -> str:
    return sys.platform


_CLOUD_USER_DATA = dedent(
    """\
    #cloud-config
    manage_etc_hosts: true
    package_update: false
    growpart:
        mode: growpart
        devices: ["/"]
        ignore_growroot_disabled: false
    write_files:
        - path: /root/.bashrc
          permissions: 0644
          content: |
            export SNAPCRAFT_BUILD_ENVIRONMENT=managed-host
            export PS1="\h \$(/bin/_snapcraft_prompt)# "
            export PATH=/snap/bin:$PATH
        - path: /bin/_snapcraft_prompt
          permissions: 0755
          content: |
            #!/bin/bash
            if [[ "$PWD" =~ ^$HOME.* ]]; then
                path="${PWD/#$HOME/\ ..}"
                if [[ "$path" == " .." ]]; then
                    ps1=""
                else
                    ps1="$path"
                fi
            else
                ps1="$PWD"
            fi
            echo -n $ps1
    """  # noqa: W605
)


class Provider(abc.ABC):

    _INSTANCE_PROJECT_DIR = "~/project"

    def __init__(
        self,
        *,
        project,
        echoer,
        is_ephemeral: bool = False,
        build_provider_flags: Dict[str, str] = None,
    ) -> None:
        self.project = project
        self.echoer = echoer
        self._is_ephemeral = is_ephemeral

        self.instance_name = "snapcraft-{}".format(project.info.name)

        if project.info.version:
            self.snap_filename = "{}_{}_{}.snap".format(
                project.info.name, project.info.version, project.deb_arch
            )
        else:
            self.snap_filename = "{}_{}.snap".format(
                project.info.name, project.deb_arch
            )

        self.provider_project_dir = os.path.join(
            BaseDirectory.save_data_path("snapcraft"),
            "projects",
            project.info.name,
            self._get_provider_name(),
        )

        if build_provider_flags is None:
            build_provider_flags = dict()
        self.build_provider_flags = build_provider_flags.copy()

        self._cached_home_directory: Optional[pathlib.Path] = None

    def __enter__(self):
        try:
            self.create()
        except errors.ProviderBaseError:
            # Destroy is idempotent
            self.destroy()
            raise
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.destroy()

    @classmethod
    @abc.abstractclassmethod
    def ensure_provider(cls) -> None:
        """Necessary steps to ensure the provider is correctly setup."""

    @classmethod
    @abc.abstractclassmethod
    def setup_provider(cls, *, echoer) -> None:
        """Necessary steps to install the provider on the host."""

    @classmethod
    @abc.abstractclassmethod
    def _get_provider_name(cls) -> str:
        """Return the provider name."""

    @classmethod
    @abc.abstractclassmethod
    def _get_is_snap_injection_capable(cls) -> bool:
        """Return whether the provider can install snaps from the host."""

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
    def _run(
        self, command: Sequence[str], hide_output: bool = False
    ) -> Optional[bytes]:
        """Run a command on the instance."""

    @abc.abstractmethod
    def _launch(self):
        """Launch the instance."""

    @abc.abstractmethod
    def _start(self):
        """Start an existing the instance."""

    @abc.abstractmethod
    def _push_file(self, *, source: str, destination: str) -> None:
        """Push a file into the instance."""

    @abc.abstractmethod
    def _is_mounted(self, target: str) -> bool:
        """Query if there is a mount at target mount point."""

    @abc.abstractmethod
    def _mount(self, host_source: str, target: str) -> None:
        """Mount host source directory to target mount point."""

    def mount_project(self) -> None:
        """Provider steps needed to make the project available to the instance.
        """
        target = (self._get_home_directory() / "project").as_posix()
        self._mount(self.project._project_dir, target)

        if self.build_provider_flags.get("bind_ssh"):
            self._mount_ssh()

    def _mount_prime_directory(self) -> bool:
        """Mount the host prime directory into the provider.

        :returns: True if the prime directory was already mounted.
        """
        target = (self._get_home_directory() / "prime").as_posix()
        if self._is_mounted(target):
            return True

        self._mount(self.project.prime_dir, target)
        return False

    def _mount_ssh(self) -> None:
        """Mount ~/.ssh to target."""
        src = (pathlib.Path.home() / ".ssh").as_posix()
        target = (self._get_home_directory() / ".ssh").as_posix()
        self._mount(src, target)

    def expose_prime(self) -> None:
        """Provider steps needed to expose the prime directory to the host.
        """
        os.makedirs(self.project.prime_dir, exist_ok=True)
        if not self._mount_prime_directory():
            self._run(command=["snapcraft", "clean", "--unprime"])

    def execute_step(self, step: steps.Step) -> None:
        self._run(command=["snapcraft", step.name])

    def clean(self, part_names: Sequence[str]) -> None:
        self._run(command=["snapcraft", "clean"] + list(part_names))

    def pack_project(self, *, output: Optional[str] = None) -> None:
        command = ["snapcraft", "snap"]
        if output:
            command.extend(["--output", output])
        self._run(command=command)

    def clean_project(self) -> bool:
        try:
            shutil.rmtree(self.provider_project_dir)
            return True
        except FileNotFoundError:
            return False

    @abc.abstractmethod
    def pull_file(self, name: str, destination: str, delete: bool = False) -> None:
        """
        Provider steps needed to retrieve a file from the instance, optionally
        deleting the source file after a successful retrieval.

        :param name: the remote filename.
        :type name: str
        :param destination: the local filename.
        :type destination: str
        :param delete: whether the file should be deleted.
        :type delete: bool
        """

    @abc.abstractmethod
    def shell(self) -> None:
        """Provider steps to provide a shell into the instance."""

    def launch_instance(self) -> None:
        # Check provider base and clean project if base has changed.
        if os.path.exists(self.provider_project_dir):
            self._ensure_base()

        try:
            # An ProviderStartError exception here means we need to create
            self._start()
        except errors.ProviderInstanceNotFoundError:
            # If starting failed, we need to make sure our data directory is
            # clean
            if os.path.exists(self.provider_project_dir):
                shutil.rmtree(self.provider_project_dir)
            os.makedirs(self.provider_project_dir)
            # then launch
            self._launch()
            # We need to setup snapcraft now to be able to refresh
            self._setup_snapcraft()
            # and do first boot related things
            self._run(["snapcraft", "refresh"])
        else:
            # We always setup snapcraft after a start to bring it up to speed with
            # what is on the host
            self._setup_snapcraft()

    def _ensure_base(self) -> None:
        info = self._load_info()
        provider_base = info["base"] if "base" in info else None
        if self._base_has_changed(self.project.info.get_build_base(), provider_base):
            self.echoer.warning(
                "Project base changed from {!r} to {!r}, cleaning build instance.".format(
                    provider_base, self.project.info.get_build_base()
                )
            )
            self.clean_project()

    def _setup_snapcraft(self) -> None:
        self._save_info(base=self.project.info.get_build_base())

        registry_filepath = os.path.join(
            self.provider_project_dir, "snap-registry.yaml"
        )

        # We do not want to inject from the host if not running from the snap
        # or if the provider cannot handle snap mounts.
        # This latter problem should go away when API for retrieving snaps
        # through snapd is generally available.
        if self._get_is_snap_injection_capable():
            inject_from_host = common.is_snap()
        else:
            inject_from_host = False

        snap_injector = SnapInjector(
            registry_filepath=registry_filepath,
            snap_arch=self.project.deb_arch,
            runner=self._run,
            file_pusher=self._push_file,
            inject_from_host=inject_from_host,
        )

        # Note that snap injection order is important.
        # If the build base is core, we do not need to inject snapd.
        # Check for None as this build can be driven from a non snappy enabled
        # system, so we may find ourselves in a situation where the base is not
        # set like on OSX or Windows.
        build_base = self.project.info.get_build_base()
        if build_base is not None and build_base != "core":
            snap_injector.add(snap_name="snapd")

        # Prevent injecting core18 twice.
        if build_base is not None and build_base != "core18":
            snap_injector.add(snap_name=build_base)

        # Inject snapcraft
        snap_injector.add(snap_name="core18")
        snap_injector.add(snap_name="snapcraft")

        snap_injector.apply()

    def _get_cloud_user_data(self) -> str:
        cloud_user_data_filepath = os.path.join(
            self.provider_project_dir, "user-data.yaml"
        )
        if os.path.exists(cloud_user_data_filepath):
            return cloud_user_data_filepath

        with open(cloud_user_data_filepath, "w") as cloud_user_data_file:
            print(_CLOUD_USER_DATA, file=cloud_user_data_file, end="")

        return cloud_user_data_filepath

    def _get_env_command(self) -> Sequence[str]:
        """Get command sequence for `env` with configured flags."""

        env_list = ["env"]

        # Configure SNAPCRAFT_HAS_TTY.
        has_tty = str(sys.stdout.isatty())
        env_list.append(f"SNAPCRAFT_HAS_TTY={has_tty}")

        # Pass through configurable environment variables.
        for key in ["http_proxy", "https_proxy"]:
            value = self.build_provider_flags.get(key)
            if not value:
                continue

            # Ensure item is treated as string and append it.
            value = str(value)
            env_list.append(f"{key}={value}")

        return env_list

    def _get_home_directory(self) -> pathlib.Path:
        """Get user's home directory path."""
        if self._cached_home_directory is not None:
            return self._cached_home_directory

        command = ["printenv", "HOME"]
        run_output = self._run(command=command, hide_output=True)

        # Shouldn't happen, but due to _run()'s return type as being Optional,
        # we need to check for it anyways for mypy.
        if not run_output:
            provider_name = self._get_provider_name()
            raise errors.ProviderExecError(
                provider_name=provider_name, command=command, exit_code=2
            )

        cached_home_directory = pathlib.Path(run_output.decode().strip())

        self._cached_home_directory = cached_home_directory
        return cached_home_directory

    def _base_has_changed(self, base: str, provider_base: str) -> bool:
        # Make it backwards compatible with instances without project info
        if base == "core18" and provider_base is None:
            return False
        elif base != provider_base:
            return True

        return False

    def _load_info(self) -> Dict[str, Any]:
        filepath = os.path.join(self.provider_project_dir, "project-info.yaml")
        if not os.path.exists(filepath):
            return dict()

        with open(filepath) as info_file:
            return yaml_utils.load(info_file)

    def _log_run(self, command: Sequence[str]) -> None:
        cmd_string = " ".join([shlex.quote(c) for c in command])
        logger.debug(f"Running: {cmd_string}")

    def _save_info(self, **data: Dict[str, Any]) -> None:
        filepath = os.path.join(self.provider_project_dir, "project-info.yaml")

        dirpath = os.path.dirname(filepath)
        if dirpath:
            os.makedirs(dirpath, exist_ok=True)

        with open(filepath, "w") as info_file:
            yaml_utils.dump(data, stream=info_file)
