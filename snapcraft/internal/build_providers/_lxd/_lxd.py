# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019-2020 Canonical Ltd
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

import logging
import os
import pathlib
import shlex
import subprocess
import sys
from textwrap import dedent
from time import sleep
from typing import Any, Dict, List, Optional, Sequence

import yaml

from snapcraft import file_utils
from snapcraft.internal import repo
from snapcraft.internal.errors import SnapcraftEnvironmentError

from .._base_provider import Provider, errors

logger = logging.getLogger(__name__)

_LXD_BUILDD_REMOTE_NAME = "snapcraft-buildd-images"
_LXD_BUILDD_IMAGES = {
    "core": "16.04",
    "core16": "16.04",
    "core18": "18.04",
    "core20": "20.04",
}


class LXC:
    """LXC Wrapper."""

    def __init__(self, *, remote: str, name: str) -> None:
        """Manage instance via `lxc`.

        :param remote: name of instance remote.
        :param name: name of instance.
        """
        self.remote = remote
        self.name = name
        self.lxc_command = file_utils.get_host_tool_path(
            command_name="lxc", package_name="lxd"
        )
        self.long_name = self.remote + ":" + self.name

    def add_remote(self, name: str, addr: str, protocol: str = "simplestreams") -> None:
        """Add a public remote."""
        self._lxc_run(
            ["remote", "add", name, addr, f"--protocol={protocol}"], check=True,
        )

    def delete(self):
        """Purge instance."""
        self._lxc_run(["delete", self.long_name, "--force"], check=True)

    def execute(self, command: List[str]):
        """Execute command in instance, allowing output to console."""
        return self._lxc_run(["exec", self.long_name, "--", *command], check=True)

    def execute_check_output(self, command: List[str]):
        """Execute command in instance, capturing output."""
        return self._lxc_run(
            ["exec", self.long_name, "--", *command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True,
        )

    def execute_popen(self, command: List[str], **popen_kwargs):
        """Execute command in instance, using Popen."""
        command = [str(self.lxc_command), "exec", self.long_name, "--", *command]
        quoted = " ".join([shlex.quote(c) for c in command])
        logger.info(f"Running: {quoted}")
        return subprocess.Popen(command, **popen_kwargs)

    def get_instances(self) -> List[Dict[str, Any]]:
        """Get instance state information."""
        proc = self._lxc_run(
            ["list", "--format=yaml", self.long_name],
            stdout=subprocess.PIPE,
            check=True,
        )

        return yaml.load(proc.stdout, Loader=yaml.FullLoader)

    def get_instance_state(self) -> Optional[Dict[str, Any]]:
        """Get instance state, if instance exists."""
        instances = self.get_instances()
        if not instances:
            return None

        for instance in self.get_instances():
            if instance["name"] == self.name:
                return instance
        return None

    def get_remotes(self) -> Dict[str, Any]:
        """Get list of remotes.

        :returns: dictionary with remote name mapping to config.
        """
        proc = self._lxc_run(
            ["remote", "list", "--format=yaml"], stdout=subprocess.PIPE, check=True,
        )
        return yaml.load(proc.stdout, Loader=yaml.FullLoader)

    def get_server_config(self) -> Dict[str, Any]:
        """Get server config that instance is running on."""
        proc = self._lxc_run(
            ["info", self.remote + ":"], stdout=subprocess.PIPE, check=True,
        )
        return yaml.load(proc.stdout, Loader=yaml.FullLoader)

    def launch(self, *, image_remote: str, image_name: str) -> None:
        """Launch instance."""
        self._lxc_run(
            ["launch", ":".join([image_remote, image_name]), self.long_name],
            check=True,
        )

    def is_instance_running(self) -> bool:
        """Check if instance is running."""
        instance_state = self.get_instance_state()
        if instance_state is None:
            return False

        return instance_state["status"] == "Running"

    def map_uid_to_root(self, uid: int) -> None:
        """Map specified uid on host to root in container."""
        self.set_config_key(
            key="raw.idmap", value=f"both {uid!s} 0",
        )

    def mount(self, device_name: str, source: str, path: str) -> None:
        """Mount host source directory to target mount point."""
        self._lxc_run(
            [
                "config",
                "device",
                "add",
                self.long_name,
                device_name,
                f"source={source}",
                f"path={path}",
            ],
            check=True,
        )

    def set_config_key(self, key: str, value: str) -> None:
        """Set instance configuration key."""
        self._lxc_run(["config", "set", self.long_name, key, value], check=True)

    def pull_file(self, *, source: str, destination: str) -> None:
        """Get file from instance."""
        self._lxc_run(
            ["file", "pull", self.long_name + source, destination], check=True,
        )

    def push_file(self, *, source: str, destination: str) -> None:
        """Push file to instance."""
        self._lxc_run(
            ["file", "push", source, self.long_name + destination], check=True,
        )

    def start(self) -> None:
        """Start container."""
        self._lxc_run(["start", self.long_name], check=True)

    def stop(self) -> None:
        """Stop container."""
        self._lxc_run(["stop", self.long_name], check=True)

    def _lxc_run(self, lxc_args: List[str], **kwargs):
        command = [str(self.lxc_command), *lxc_args]
        quoted = " ".join([shlex.quote(c) for c in command])
        logger.info(f"Running: {quoted}")
        return subprocess.run(command, **kwargs)


class LXD(Provider):
    """A LXD provider for snapcraft to execute its lifecycle."""

    _PROJECT_DEVICE_NAME = "snapcraft-project"
    _PROJECT_EXPORTED_PRIME_NAME = "snapcraft-project-prime"

    # Given that we are running snapcraft from the snapcraft snap, which has
    # classic confinement and require using the lxd snap, the lxd and lxc
    # binaries should be found in /snap/bin
    _LXD_BIN = os.path.join(os.path.sep, "snap", "bin", "lxd")

    @classmethod
    def ensure_provider(cls):
        error_message = None  # type: Optional[str]
        prompt_installable = False

        if sys.platform != "linux":
            error_message = "LXD is not supported on this platform"
        else:
            try:
                if not repo.snaps.SnapPackage.is_snap_installed("lxd"):
                    error_message = (
                        "The LXD snap is required to continue: snap install lxd"
                    )
                    prompt_installable = True
            except repo.errors.SnapdConnectionError:
                error_message = (
                    "snap support is required to continue: "
                    "https://docs.snapcraft.io/installing-snapd/6735"
                )

        if error_message is not None:
            raise errors.ProviderNotFound(
                provider=cls._get_provider_name(),
                prompt_installable=prompt_installable,
                error_message=error_message,
            )

        # If we reach this point, it means the lxd snap is properly setup.
        # Now is the time for additional sanity checks to ensure the provider
        # will work.
        try:
            # TODO: add support for more distributions. Maybe refactor a bit so that Repo behaves
            # similar to a build provider.
            if repo.Repo.is_package_installed("lxd") or repo.Repo.is_package_installed(
                "lxd-client"
            ):
                raise SnapcraftEnvironmentError(
                    (
                        "The {!r} provider does not support having the 'lxd' or "
                        "'lxd-client' deb packages installed. To completely migrate "
                        "to the LXD snap run 'lxd.migrate' and try again."
                    ).format(cls._get_provider_name())
                )
        except repo.errors.NoNativeBackendError:
            pass

    @classmethod
    def setup_provider(cls, *, echoer) -> None:
        repo.snaps.install_snaps(["lxd/latest/stable"])

        try:
            subprocess.check_output([cls._LXD_BIN, "waitready", "--timeout=30"])
        except subprocess.CalledProcessError as call_error:
            raise SnapcraftEnvironmentError(
                "Timeout reached waiting for LXD to start."
            ) from call_error

        try:
            subprocess.check_output([cls._LXD_BIN, "init", "--auto"])
        except subprocess.CalledProcessError as call_error:
            raise SnapcraftEnvironmentError(
                "Failed to initialize LXD. "
                "Try manually initializing before trying again: lxd init --auto."
            ) from call_error

    @classmethod
    def _get_provider_name(cls):
        return "LXD"

    @classmethod
    def _get_is_snap_injection_capable(cls) -> bool:
        return True

    def __init__(
        self,
        *,
        project,
        echoer,
        build_provider_flags: Dict[str, str],
        is_ephemeral: bool = False,
        lxd_image_remote_addr: str = "https://cloud-images.ubuntu.com/buildd/releases",
        lxd_image_remote_name: str = "snapcraft-buildd-images",
        lxd_image_remote_protocol: str = "simplestreams",
    ) -> None:
        super().__init__(
            project=project,
            echoer=echoer,
            is_ephemeral=is_ephemeral,
            build_provider_flags=build_provider_flags,
        )

        # TODO: I regret using generalized build_provider_flags.
        self._lxd_remote: str = build_provider_flags.get(
            "SNAPCRAFT_LXD_REMOTE", "local"
        )

        self._lxd_image_remote_addr = lxd_image_remote_addr
        self._lxd_image_remote_name = lxd_image_remote_name
        self._lxd_image_remote_protocol = lxd_image_remote_protocol

        # TODO: Add build_base an parameter, removing project.
        build_base = project._get_build_base()
        lxd_image_name = _LXD_BUILDD_IMAGES.get(build_base)
        if lxd_image_name is None:
            raise errors.ProviderInvalidBaseError(
                provider_name=self._get_provider_name(), build_base=build_base
            )
        self._lxd_image_name = lxd_image_name

        self._lxc = LXC(remote=self._lxd_remote, name=self.instance_name)

    def _run(
        self, command: Sequence[str], hide_output: bool = False
    ) -> Optional[bytes]:
        env_command = super()._get_env_command()

        if hide_output:
            proc = self._lxc.execute_check_output([*env_command, *command])
            return proc.stdout

        self._lxc.execute([*env_command, *command])
        return None

    def _configure_image_remote(self) -> None:
        remotes = self._lxc.get_remotes()
        matching_remote = remotes.get(self._lxd_image_remote_name)
        if matching_remote:
            if (
                matching_remote.get("addr") != self._lxd_image_remote_addr
                or matching_remote.get("protocol") != self._lxd_image_remote_protocol
            ):
                raise SnapcraftEnvironmentError(
                    f"Unexpected LXD remote: {matching_remote!r}"
                )
            return

        self._lxc.add_remote(
            name=self._lxd_image_remote_name,
            addr=self._lxd_image_remote_addr,
            protocol=self._lxd_image_remote_protocol,
        )

    def _enable_instance_mknod(self) -> None:
        """Enable mknod in container, if possible.

        See: https://linuxcontainers.org/lxd/docs/master/syscall-interception
        """
        cfg = self._lxc.get_server_config()
        env = cfg.get("environment", dict())
        kernel_features = env.get("kernel_features", dict())
        seccomp_listener = kernel_features.get("seccomp_listener", "false")

        if seccomp_listener == "true":
            self._lxc.set_config_key(
                key="security.syscalls.intercept.mknod", value="true",
            )

    def _launch(self) -> None:
        self._configure_image_remote()
        self._lxc.launch(
            image_remote=self._lxd_image_remote_name, image_name=self._lxd_image_name
        )
        self._enable_instance_mknod()

    def _start(self):
        instance_state = self._lxc.get_instance_state()
        if instance_state is None:
            raise errors.ProviderInstanceNotFoundError(instance_name=self.instance_name)

        if instance_state["status"] != "Running":
            self._lxc.start()
        self.echoer.wrapped("Waiting for container to be ready")

    def _push_file(self, *, source: str, destination: str) -> None:
        self._lxc.push_file(source=source, destination=destination)

    def _sync_project(self) -> None:
        """Naive sync to remote using tarball."""

        logger.info("Syncing project to remote container...")
        target_path = self._get_target_project_directory()

        self._lxc.execute(["rm", "-rf", target_path])
        self._lxc.execute(["mkdir", "-p", target_path])

        tar_path = file_utils.get_host_tool_path(command_name="tar", package_name="tar")
        archive_proc = subprocess.Popen(
            [tar_path, "cpf", "-", "-C", self.project._project_dir, "."],
            stdout=subprocess.PIPE,
        )

        target_proc = self._lxc.execute_popen(
            ["tar", "xpvf", "-", "-C", target_path], stdin=archive_proc.stdout,
        )

        # Allow archive_proc to receive a SIGPIPE if target_proc exits.
        if archive_proc.stdout:
            archive_proc.stdout.close()

        # Waot until done.
        target_proc.communicate()

    def create(self) -> None:
        """Create the LXD instance and setup the build environment."""
        self.echoer.info("Launching a container.")
        self.launch_instance()
        if self._lxd_remote == "local":
            self._mount_project()
        else:
            self._sync_project()

    def destroy(self) -> None:
        """Destroy the instance, trying to stop it first."""
        self._lxc.stop()

    def _get_mount_name(self, target: str) -> str:
        """Provide a formatted name for target mount point."""
        home_dir = self._get_home_directory().as_posix()

        # Special cases for compatibility.
        if target == os.path.join(home_dir, "project"):
            return self._PROJECT_DEVICE_NAME
        elif target == os.path.join(home_dir, "prime"):
            return self._PROJECT_EXPORTED_PRIME_NAME

        # Replace home directory with "snapcraft".
        name = target.replace(home_dir, "snapcraft", 1)

        # Replace path separators with dashes.
        name = name.replace("/", "-")
        return name

    def _is_mounted(self, target: str) -> bool:
        """Query if there is a mount at target mount point."""
        device_name = self._get_mount_name(target)

        instance_state = self._lxc.get_instance_state()
        if instance_state is None:
            return False

        instance_devices = instance_state.get("devices")
        if not instance_devices:
            return False

        return device_name in instance_devices

    def _mount(self, host_source: str, target: str) -> None:
        """Mount host source directory to target mount point."""
        device_name = self._get_mount_name(target)

        self._lxc.mount(device_name=device_name, source=host_source, path=target)

    def clean_project(self) -> bool:
        was_cleaned = super().clean_project()

        instance_state = self._lxc.get_instance_state()
        if instance_state:
            self._lxc.delete()
            was_cleaned = True

        return was_cleaned

    def pull_file(self, name: str, destination: str, delete: bool = False) -> None:
        self._lxc.pull_file(source=name, destination=destination)
        if delete:
            self._lxc.execute(["rm", "-f", name])

    def shell(self) -> None:
        self._lxc.execute(["/bin/bash"])

    def snap(self) -> List[pathlib.Path]:
        """Naive snap implementation to gather built snaps.

        Naive enough to grab anything with the .snap, whether
        it was built or not.
        """
        target_path = self._get_target_project_directory()
        proc = self._lxc.execute_check_output(
            ["bash", "-c", f"ls -1 {target_path}/*.snap"]
        )

        snap_paths = [
            pathlib.Path(self.project._project_dir, pathlib.Path(p).name)
            for p in proc.stdout.decode().strip().split("\n")
        ]

        for dst in snap_paths:
            src = pathlib.Path(target_path, pathlib.Path(dst).name)
            self._lxc.pull_file(source=str(src), destination=str(dst))

        return snap_paths

    def _wait_for_systemd(self) -> None:
        # systemctl states we care about here are:
        # - running: The system is fully operational. Process returncode: 0
        # - degraded: The system is operational but one or more units failed.
        #             Process returncode: 1
        for i in range(40):
            try:
                self._lxc.execute_check_output(["systemctl", "is-system-running"])
                break
            except subprocess.CalledProcessError as error:
                if "degraded" in error.stdout:
                    break
                logger.debug(f"systemctl is-system-running: {error.stdout!r}")
                sleep(0.5)
        else:
            self.echoer.warning("Timed out waiting for systemd to be ready...")

    def _wait_for_network(self) -> None:
        self.echoer.wrapped("Waiting for network to be ready...")
        for i in range(40):
            try:
                self._lxc.execute_check_output(["getent", "hosts", "snapcraft.io"])
                break
            except errors.ProviderExecError:
                sleep(0.5)
        else:
            self.echoer.warning("Failed to setup networking.")

    def _setup_environment(self) -> None:
        super()._setup_environment()

        self._install_file(
            path="/etc/systemd/network/10-eth0.network",
            content=dedent(
                """
                [Match]
                Name=eth0

                [Network]
                DHCP=ipv4
                LinkLocalAddressing=ipv6

                [DHCP]
                RouteMetric=100
                UseMTU=true
                """
            ),
            permissions="0644",
        )

        self._install_file(
            path="/etc/hostname", content=self.instance_name, permissions="0644"
        )

        self._wait_for_systemd()

        # Use resolv.conf managed by systemd-resolved.
        self._lxc.execute_check_output(
            ["ln", "-sf", "/run/systemd/resolve/resolv.conf", "/etc/resolv.conf"]
        )

        self._lxc.execute_check_output(["systemctl", "enable", "systemd-resolved"])
        self._lxc.execute_check_output(["systemctl", "enable", "systemd-networkd"])

        self._lxc.execute_check_output(["systemctl", "restart", "systemd-resolved"])
        self._lxc.execute_check_output(["systemctl", "restart", "systemd-networkd"])

        self._wait_for_network()

        # Setup snapd to bootstrap.
        self._lxc.execute(["apt-get", "update"])

        # First install fuse and udev, snapd requires them.
        # Snapcraft requires dirmngr
        self._lxc.execute(["apt-get", "install", "dirmngr", "udev", "fuse", "--yes"])

        # the system needs networking
        self._lxc.execute_check_output(["systemctl", "enable", "systemd-udevd"])
        self._lxc.execute_check_output(["systemctl", "start", "systemd-udevd"])

        # And only then install snapd.
        self._lxc.execute(["apt-get", "install", "snapd", "sudo", "--yes"])
        self._lxc.execute_check_output(["systemctl", "start", "snapd"])

    def _setup_snapcraft(self):
        self._wait_for_network()
        super()._setup_snapcraft()
