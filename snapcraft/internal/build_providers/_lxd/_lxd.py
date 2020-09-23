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

import yaml
import logging
import os
import stat
import subprocess
import sys
import tempfile
from textwrap import dedent
from time import sleep
from typing import Any, Dict, List, Optional, Sequence

from snapcraft import file_utils
from snapcraft.internal import repo
from snapcraft.internal.errors import SnapcraftEnvironmentError

from .._base_provider import Provider, errors
from ._images import get_image_source

logger = logging.getLogger(__name__)
# Filter out attribute setting warnings for properties that exist in LXD operations
# but are unhandled in pylxd.


class LXC:
    """LXC Instance Manager."""

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

    def _enable_mknod(self) -> None:
        """Enable mknod in container, if possible.

        See: https://linuxcontainers.org/lxd/docs/master/syscall-interception
        """
        cfg = self.get_server_config()
        env = cfg.get("environment", dict())
        kernel_features = env.get("kernel_features", dict())
        seccomp_listener = kernel_features.get("seccomp_listener", "false")

        if seccomp_listener == "true":
            self.config_set(
                key="security.syscalls.intercept.mknod", value="true",
            )

    def execute(self, command: List[str]):
        """Execute command in instance, allowing output to console."""
        subprocess.run([self.lxc_command, "exec", self.long_name, "--", *command])

    def execute_check_output(self, command: List[str]):
        """Execute command in instance, capturing output."""
        proc = subprocess.run(
            [self.lxc_command, "exec", self.long_name, "--", *command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return proc.stdout, proc.stderr

    def launch(self, image: str) -> None:
        proc = subprocess.run(
            [self.lxc_command, "launch", image, self.long_name],
            stdout=subprocess.PIPE,
        )

    def get_state(self) -> Dict[str, Any]:
        """Get instance state information."""
        proc = subprocess.run(
            [self.lxc_command, "list", "--format=yaml", self.log_name],
            stdout=subprocess.PIPE,
        )
        return yaml.load(proc.stdout)[0]

    def get_config(self) -> Dict[str, Any]:
        """Get instance configuration."""
        return self.get_state()["config"]

    def get_config_key(self, key: str) -> str:
        """Get instance configuration key."""
        proc = subprocess.run(
            [self.lxc_command, "config", "get", self.long_name, key],
            stdout=subprocess.PIPE,
        )
        return proc.stdout

    def get_server_config(self) -> Dict[str, Any]:
        """Get server config that instance is running on."""
        proc = subprocess.run(
            [self.lxc_command, "info", self.remote + ":"], stdout=subprocess.PIPE,
        )
        return yaml.load(proc.stdout)

    def launch(self, image_remote: str, image_name: str) -> None:
        """Launch instance."""
        subprocess.run(
            [self.lxc_command, launch, ":".join(image_remote, image_name), self.long_name]
        )

    def is_instance_running(self) -> bool:
        """Check if instance is running."""
        return self.get_state()["status"] == "Running"

    def _map_uid_to_root(self, uid: int) -> None:
        """Map specified uid on host to root in container."""
        self._lxc.config_set(
            key="raw.idmap", value=f"both {uid!s} 0",
        )

    def set_config_key(self, key: str, value: str) -> None:
        """Set instance configuration key."""
        subprocess.run([self.lxc_command, "config", "set", self.long_name, key, value])

    def pull_file(self, *, source: str, destination: str) -> None:
        """Get file from instance."""
        subprocess.run([self.lxc_command, "pull", self.long_name + source, destination])

    def push_file(self, *, source: str, destination: str) -> None:
        """Push file to instance."""
        subprocess.run([self.lxc_command, "push", source, self.long_name + destination])

    def start(self) -> None:
        """Start container."""
        subprocess.run([self.lxc_command, "start", self.long_name])

    def stop(self) -> None:
        """Stop container."""
        subprocess.run([self.lxc_command, "stop", self.long_name])


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
    ) -> None:
        super().__init__(
            project=project,
            echoer=echoer,
            is_ephemeral=is_ephemeral,
            build_provider_flags=build_provider_flags,
        )

        self._lxd_remote: str = build_provider_flags.get(
            "SNAPCRAFT_LXD_REMOTE", "local"
        )

        self._lxc = LXC(remote=self._lxd_remote, name=self.instance_name)

    def _run(self, command: Sequence[str], hide_output: bool = False) -> Optional[bytes]:
        env_command = super()._get_env_command()

        if hide_output:
            self._lxc.execute_check_output([*env_command, *command])
        else:
            self._lxc.execute([*env_command, *command])

    def _launch(self) -> None:
        build_base = self.project._get_build_base()
        try:
            source = get_image_source(base=build_base)
        except KeyError:
            raise errors.ProviderInvalidBaseError(
                provider_name=self._get_provider_name(), build_base=build_base
            )

        self._lxc_lxc.execute_check_output(["launch", source, self._instance_name])

    def _start(self):
        self.lxc.start()
        self.echoer.wrapped("Waiting for container to be ready")

    def _push_file(self, *, source: str, destination: str) -> none:
        self._lxc.push_file(source=source, destination=destination)

    def _sync_project(self) -> None:
        logger.info("Syncing project to remote container...")
        with tempfile.NamedTemporaryFile(
            mode="w", suffix="fake-ssh", delete=False
        ) as f:
            f.write(
                dedent(
                    f"""
                #!/bin/sh -x
                shift
                exec lxc exec -T {self.instance_name} -- "$@"
                """
                )
            )

            fake_ssh_path = f.name

        st = os.stat(fake_ssh_path)
        os.chmod(fake_ssh_path, st.st_mode | stat.S_IEXEC)

        rsync_path = file_utils.get_host_tool_path(
            command_name="rsync", package_name="rsync"
        )
        target_path = self._get_target_project_directory()

        try:
            subprocess.run(
                [
                    rsync_path,
                    "-avPz",
                    "-e",
                    fake_ssh_path,
                    self.project._project_dir,
                    f"remote_container:{target_path}",
                ]
            )
        except subprocess.CalledProcessError as error:
            raise RuntimeError(
                f"Failed to rsync to remote container: {error.stdout} {error.stderr}"
            )

        os.unlink(fake_ssh_path)

    def create(self) -> None:
        """Create the LXD instance and setup the build environment."""
        self.echoer.info("Launching a container.")
        self.launch_instance()
        if self._lxd_remote != "local":
            self._mount_project()
        else:
            self._sync_project()

    def destroy(self) -> None:
        """Destroy the instance, trying to stop it first."""
        self._stop()

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
        # Sanity check - developer error if container not initialized.
        if self._container is None:
            raise RuntimeError("Attempted to use container before starting.")

        name = self._get_mount_name(target)
        return name in self._container.devices

    def _mount(self, host_source: str, target: str) -> None:
        """Mount host source directory to target mount point."""
        # Sanity check - developer error if container not initialized.
        if self._container is None:
            raise RuntimeError("Attempted to use container before starting.")

        if self._is_mounted(target):
            # Nothing to do if already mounted.
            return

        name = self._get_mount_name(target)
        self._container.sync()
        self._container.devices[name] = {
            "type": "disk",
            "source": host_source,
            "path": target,
        }

        try:
            self._container.save(wait=True)
        except pylxd.exceptions.LXDAPIException as lxd_api_error:
            raise errors.ProviderMountError(
                provider_name=self._get_provider_name(), error_message=lxd_api_error
            ) from lxd_api_error

    def clean_project(self) -> bool:
        if
        was_cleaned = super().clean_project()
        if was_cleaned:
            if self._container is None:
                try:
                    self._container = self._lxd_client.containers.get(
                        self.instance_name
                    )
                except pylxd.exceptions.NotFound:
                    # If no container found, nothing to delete.
                    return was_cleaned
            self._stop()
            self._container.delete(wait=True)
        return was_cleaned

    def pull_file(self, name: str, destination: str, delete: bool = False) -> None:
        self._lxc.pull_file(source=name, destination=destination)
        if delete:
            self._lxc.execute(["rm", "-f", source])

    def shell(self) -> None:
        self._lxc.execute(["/bin/bash"])

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
                logger.debug(f"systemctl is-system-running: {running_state!r}")
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
