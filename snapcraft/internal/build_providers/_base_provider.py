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
import base64
import logging
import os
import pathlib
import pkg_resources
import platform
import shlex
import shutil
import sys
import tempfile
from textwrap import dedent
from typing import Optional, Sequence
from typing import Any, Dict

from xdg import BaseDirectory

import snapcraft
from . import errors
from ._snap import SnapInjector
from snapcraft.internal import common, steps
from snapcraft import yaml_utils


logger = logging.getLogger(__name__)


def _get_platform() -> str:
    return sys.platform


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

        self.instance_name = "snapcraft-{}".format(project._snap_meta.name)

        if project._snap_meta.version:
            self.snap_filename = "{}_{}_{}.snap".format(
                project._snap_meta.name, project._snap_meta.version, project.deb_arch
            )
        else:
            self.snap_filename = "{}_{}.snap".format(
                project._snap_meta.name, project.deb_arch
            )

        self.provider_project_dir = os.path.join(
            BaseDirectory.save_data_path("snapcraft"),
            "projects",
            project._snap_meta.name,
            self._get_provider_name(),
        )

        if build_provider_flags is None:
            build_provider_flags = dict()
        self.build_provider_flags = build_provider_flags.copy()

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

        if self.build_provider_flags.get("SNAPCRAFT_BIND_SSH"):
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
        # Clean project if we cannot trust existing environment, or it
        # no longer matches project target.
        if os.path.exists(self.provider_project_dir):
            self._ensure_compatible_build_environment()

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

            # Configure environment for snapcraft use.
            self._setup_environment()

            # Refresh repository caches.
            self._run(["apt-get", "update"])

            # And make sure we are using the latest from that cache.
            self._run(["apt-get", "dist-upgrade", "--yes"])

        # We always setup snapcraft after a start to bring it up to speed with
        # what is on the host
        self._setup_snapcraft()

    def _check_environment_needs_cleaning(self) -> bool:
        info = self._load_info()
        provider_base = info.get("base")
        built_by = info.get("created-by-snapcraft-version")
        build_base = self.project._get_build_base()

        if provider_base is None or built_by is None:
            self.echoer.warning(
                "Build environment is in unknown state, cleaning first."
            )
            return True
        elif build_base != provider_base:
            self.echoer.warning(
                f"Project base changed from {provider_base!r} to {build_base!r}, cleaning first."
            )
            return True
        elif pkg_resources.parse_version(
            snapcraft._get_version()
        ) < pkg_resources.parse_version(built_by):
            self.echoer.warning(
                f"Build environment was created with newer snapcraft version {built_by!r}, cleaning first."
            )
            return True

        return False

    def _ensure_compatible_build_environment(self) -> None:
        """Force clean of build-environment if project is not compatible."""

        if self._check_environment_needs_cleaning():
            self.clean_project()

    def _install_file(self, *, path: str, content: str, permissions: str) -> None:
        basename = os.path.basename(path)

        with tempfile.NamedTemporaryFile(suffix=basename) as temp_file:
            temp_file.write(content.encode())
            temp_file.flush()
            # Push to a location that can be written to by all backends
            # with unique files depending on path.
            remote_file = os.path.join(
                "/var/tmp", base64.b64encode(path.encode()).decode()
            )
            self._push_file(source=temp_file.name, destination=remote_file)
            self._run(["mv", remote_file, path])
            # This chown is not necessarily needed. but does keep things
            # consistent.
            self._run(["chown", "root:root", path])
            self._run(["chmod", permissions, path])

    def _get_code_name_from_build_base(self):
        build_base = self.project._get_build_base()

        return {
            "core": "xenial",
            "core16": "xenial",
            "core18": "bionic",
            "core20": "focal",
        }[build_base]

    def _get_primary_mirror(self) -> str:
        primary_mirror = os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT_PRIMARY_MIRROR", None)

        if primary_mirror is None:
            if platform.machine() in ["i686", "x86_64"]:
                primary_mirror = "http://archive.ubuntu.com/ubuntu"
            else:
                primary_mirror = "http://ports.ubuntu.com/ubuntu-ports"

        return primary_mirror

    def _get_security_mirror(self) -> str:
        security_mirror = os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT_PRIMARY_MIRROR", None)

        if security_mirror is None:
            if platform.machine() in ["i686", "x86_64"]:
                security_mirror = "http://security.ubuntu.com/ubuntu"
            else:
                security_mirror = "http://ports.ubuntu.com/ubuntu-ports"

        return security_mirror

    def _setup_environment(self) -> None:
        self._install_file(
            path="/root/.bashrc",
            content=dedent(
                """\
                        #!/bin/bash
                        export PS1="\\h \\$(/bin/_snapcraft_prompt)# "
                        """
            ),
            permissions="0600",
        )

        self._install_file(
            path="/bin/_snapcraft_prompt",
            content=dedent(
                """\
                        #!/bin/bash
                        if [[ "$PWD" =~ ^$HOME.* ]]; then
                            path="${PWD/#$HOME/\\ ..}"
                            if [[ "$path" == " .." ]]; then
                                ps1=""
                            else
                                ps1="$path"
                            fi
                        else
                            ps1="$PWD"
                        fi
                        echo -n $ps1
                        """
            ),
            permissions="0755",
        )

        self._install_file(path="/etc/apt/sources.list", content="", permissions="0644")

        self._install_file(
            path="/etc/apt/sources.list.d/default.sources",
            content=dedent(
                """\
                    Types: deb deb-src
                    URIs: {primary_mirror}
                    Suites: {release} {release}-updates
                    Components: main multiverse restricted universe
                    """.format(
                    primary_mirror=self._get_primary_mirror(),
                    release=self._get_code_name_from_build_base(),
                )
            ),
            permissions="0644",
        )

        self._install_file(
            path="/etc/apt/sources.list.d/default-security.sources",
            content=dedent(
                """\
                        Types: deb deb-src
                        URIs: {security_mirror}
                        Suites: {release}-security
                        Components: main multiverse restricted universe
                        """.format(
                    security_mirror=self._get_security_mirror(),
                    release=self._get_code_name_from_build_base(),
                )
            ),
            permissions="0644",
        )

        self._install_file(
            path="/etc/apt/apt.conf.d/00-snapcraft",
            content=dedent(
                """\
                    Apt::Install-Recommends "false";
                    """
            ),
            permissions="0644",
        )

    def _setup_snapcraft(self) -> None:
        self._save_info(
            data={
                "base": self.project._get_build_base(),
                "created-by-snapcraft-version": snapcraft._get_version(),
            }
        )

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
        build_base = self.project._get_build_base()
        if build_base is not None and build_base != "core":
            snap_injector.add(snap_name="snapd")

        # Prevent injecting core18 twice.
        if build_base is not None and build_base != "core18":
            snap_injector.add(snap_name=build_base)

        # Inject snapcraft
        snap_injector.add(snap_name="core18")
        snap_injector.add(snap_name="snapcraft")

        snap_injector.apply()

    def _get_env_command(self) -> Sequence[str]:
        """Get command sequence for `env` with configured flags."""

        env_list = ["env"]

        # Tell Snapcraft it can take ownership of the host.
        env_list.append("SNAPCRAFT_BUILD_ENVIRONMENT=managed-host")

        # Set the HOME directory.
        env_list.append(f"HOME={self._get_home_directory().as_posix()}")

        # Configure SNAPCRAFT_HAS_TTY.
        has_tty = str(sys.stdout.isatty())
        env_list.append(f"SNAPCRAFT_HAS_TTY={has_tty}")

        # Pass through configurable environment variables.
        for key, value in self.build_provider_flags.items():
            if not value:
                continue

            # Ensure item is treated as string and append it.
            value = str(value)
            env_list.append(f"{key}={value}")

        return env_list

    def _get_home_directory(self) -> pathlib.Path:
        """Get user's home directory path."""
        return pathlib.Path("/root")

    def _load_info(self) -> Dict[str, Any]:
        filepath = os.path.join(self.provider_project_dir, "project-info.yaml")
        if not os.path.exists(filepath):
            return dict()

        with open(filepath) as info_file:
            return yaml_utils.load(info_file)

    def _log_run(self, command: Sequence[str]) -> None:
        cmd_string = " ".join([shlex.quote(c) for c in command])
        logger.debug(f"Running: {cmd_string}")

    def _save_info(self, *, data: Dict[str, Any]) -> None:
        filepath = os.path.join(self.provider_project_dir, "project-info.yaml")

        dirpath = os.path.dirname(filepath)
        if dirpath:
            os.makedirs(dirpath, exist_ok=True)

        with open(filepath, "w") as info_file:
            yaml_utils.dump(data, stream=info_file)
