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

from textwrap import dedent
import logging
import os
import subprocess
import sys
import urllib.parse
import warnings
from time import sleep
from typing import Dict, Optional, Sequence

from .._base_provider import Provider
from .._base_provider import errors
from ._images import get_image_source
from snapcraft.internal import repo
from snapcraft.internal.errors import SnapcraftEnvironmentError

# LXD is only supported on Linux and causes issues when imported on Windows.
# We conditionally import it and rely on ensure_provider() to check OS before
# using pylxd.
if sys.platform == "linux":
    import pylxd

logger = logging.getLogger(__name__)
# Filter out attribute setting warnings for properties that exist in LXD operations
# but are unhandled in pylxd.
warnings.filterwarnings("ignore", module="pylxd.models.operation")


class LXD(Provider):
    """A LXD provider for snapcraft to execute its lifecycle."""

    _PROJECT_DEVICE_NAME = "snapcraft-project"
    _PROJECT_EXPORTED_PRIME_NAME = "snapcraft-project-prime"

    # Given that we are running snapcraft from the snapcraft snap, which has
    # classic confinement and require using the lxd snap, the lxd and lxc
    # binaries should be found in /snap/bin
    _LXD_BIN = os.path.join(os.path.sep, "snap", "bin", "lxd")
    _LXC_BIN = os.path.join(os.path.sep, "snap", "bin", "lxc")

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
        is_ephemeral: bool = False,
        build_provider_flags: Dict[str, str] = None,
    ) -> None:
        super().__init__(
            project=project,
            echoer=echoer,
            is_ephemeral=is_ephemeral,
            build_provider_flags=build_provider_flags,
        )
        # This endpoint is hardcoded everywhere lxc/lxd-pkg-snap#33
        lxd_socket_path = "/var/snap/lxd/common/lxd/unix.socket"
        endpoint = "http+unix://{}".format(urllib.parse.quote(lxd_socket_path, safe=""))
        try:
            self._lxd_client: pylxd.Client = pylxd.Client(endpoint=endpoint)
        except pylxd.client.exceptions.ClientConnectionFailed:
            raise errors.ProviderCommunicationError(
                provider_name=self._get_provider_name(),
                message="cannot connect to the LXD socket ({!r}).".format(
                    lxd_socket_path
                ),
            )

        self._container: Optional[pylxd.models.container.Container] = None

    def _run(
        self, command: Sequence[str], hide_output: bool = False
    ) -> Optional[bytes]:
        self._ensure_container_running()

        env_command = super()._get_env_command()

        # TODO: use pylxd
        cmd = [self._LXC_BIN, "exec", self.instance_name, "--"]
        cmd.extend(env_command)
        cmd.extend(command)
        self._log_run(cmd)

        output = None
        try:
            if hide_output:
                output = subprocess.check_output(cmd)
            else:
                subprocess.check_call(cmd)
        except subprocess.CalledProcessError as process_error:
            raise errors.ProviderExecError(
                provider_name=self._get_provider_name(),
                command=command,
                exit_code=process_error.returncode,
                output=process_error.output,
            ) from process_error

        return output

    def _launch(self) -> None:
        build_base = self.project._get_build_base()
        try:
            source = get_image_source(base=build_base)
        except KeyError:
            raise errors.ProviderInvalidBaseError(
                provider_name=self._get_provider_name(), build_base=build_base
            )

        config = {"name": self.instance_name, "source": source}

        try:
            container = self._lxd_client.containers.create(config, wait=True)
        except pylxd.exceptions.LXDAPIException as lxd_api_error:
            raise errors.ProviderLaunchError(
                provider_name=self._get_provider_name(), error_message=lxd_api_error
            ) from lxd_api_error
        container.save(wait=True)
        self._container = container

        self._start()

    def _supports_syscall_interception(self) -> bool:
        # syscall interception relies on the seccomp_listener kernel feature
        environment = self._lxd_client.host_info.get("environment", {})
        kernel_features = environment.get("kernel_features", {})
        return kernel_features.get("seccomp_listener", "false") == "true"

    def _start(self):
        if not self._lxd_client.containers.exists(self.instance_name):
            raise errors.ProviderInstanceNotFoundError(instance_name=self.instance_name)

        if self._container is None:
            self._container = self._lxd_client.containers.get(self.instance_name)

        self._container.sync()

        # map to the owner of the directory we are eventually going to write the
        # snap to.
        self._container.config["raw.idmap"] = "both {!s} 0".format(
            os.stat(self.project._project_dir).st_uid
        )
        # If possible, allow container to make safe mknod calls. Documented at
        # https://linuxcontainers.org/lxd/docs/master/syscall-interception
        if self._supports_syscall_interception():
            self._container.config["security.syscalls.intercept.mknod"] = "true"
        self._container.save(wait=True)

        if self._container.status.lower() != "running":
            try:
                self._container.start(wait=True)
            except pylxd.exceptions.LXDAPIException as lxd_api_error:
                print(self._container.status)
                raise errors.ProviderStartError(
                    provider_name=self._get_provider_name(), error_message=lxd_api_error
                ) from lxd_api_error

        self.echoer.wrapped("Waiting for container to be ready")

    def _stop(self):
        # If _container is still None here it means creation/starting was not
        # successful.
        if self._container is None:
            return

        self._container.sync()

        if self._container.status.lower() != "stopped":
            try:
                self._container.stop(wait=True)
            except pylxd.exceptions.LXDAPIException as lxd_api_error:
                raise errors.ProviderStopError(
                    provider_name=self._get_provider_name(), error_message=lxd_api_error
                ) from lxd_api_error

    def _push_file(self, *, source: str, destination: str) -> None:
        # Sanity check - developer error if container not initialized.
        if self._container is None:
            raise RuntimeError("Attempted to use container before starting.")

        self._ensure_container_running()

        # TODO: better handling of larger files.
        with open(source, "rb") as source_data:
            source_contents = source_data.read()

        try:
            self._container.files.put(destination, source_contents)
        except pylxd.exceptions.LXDAPIException as lxd_api_error:
            raise errors.ProviderFileCopyError(
                provider_name=self._get_provider_name(), error_message=lxd_api_error
            )

    def create(self) -> None:
        """Create the LXD instance and setup the build environment."""
        self.echoer.info("Launching a container.")
        self.launch_instance()

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
        # Sanity check - developer error if container not initialized.
        if self._container is None:
            raise RuntimeError("Attempted to use container before starting.")

        self._ensure_container_running()

        # TODO: better handling of larger files.
        try:
            source_data = self._container.files.get(name)
        except pylxd.exceptions.LXDAPIException as lxd_api_error:
            raise errors.ProviderFileCopyError(
                provider_name=self._get_provider_name(), error_message=lxd_api_error
            )
        else:
            with open(destination, "wb") as destination_data:
                destination_data.write(source_data)
        if delete and self._container.files.delete_available:
            self._container.files.delete(name)
        if delete and not self._container.files.delete_available:
            logger.warning("File deletion not supported by this LXD version.")

    def shell(self) -> None:
        self._run(command=["/bin/bash"])

    def _wait_for_systemd(self) -> None:
        # systemctl states we care about here are:
        # - running: The system is fully operational. Process returncode: 0
        # - degraded: The system is operational but one or more units failed.
        #             Process returncode: 1
        for i in range(40):
            try:
                self._run(["systemctl", "is-system-running"], hide_output=True)
                break
            except errors.ProviderExecError as exec_error:
                if exec_error.output is not None:
                    running_state = exec_error.output.decode().strip()
                    if running_state == "degraded":
                        break
                    logger.debug(f"systemctl is-system-running: {running_state!r}")
                sleep(0.5)
        else:
            self.echoer.warning("Timed out waiting for systemd to be ready...")

    def _wait_for_network(self) -> None:
        self.echoer.wrapped("Waiting for network to be ready...")
        for i in range(40):
            try:
                self._run(["getent", "hosts", "snapcraft.io"], hide_output=True)
                break
            except errors.ProviderExecError:
                sleep(0.5)
        else:
            self.echoer.warning("Failed to setup networking.")

    def _setup_environment(self) -> None:
        if self._container is None:
            raise RuntimeError("Attempted to use container before starting.")

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
        self._run(["ln", "-sf", "/run/systemd/resolve/resolv.conf", "/etc/resolv.conf"])

        self._run(["systemctl", "enable", "systemd-resolved"])
        self._run(["systemctl", "enable", "systemd-networkd"])

        self._run(["systemctl", "restart", "systemd-resolved"])
        self._run(["systemctl", "restart", "systemd-networkd"])

        self._wait_for_network()

        # Setup snapd to bootstrap.
        self._run(["apt-get", "update"])

        # First install fuse and udev, snapd requires them.
        # Snapcraft requires dirmngr
        self._run(["apt-get", "install", "dirmngr", "udev", "fuse", "--yes"])

        # the system needs networking
        self._run(["systemctl", "enable", "systemd-udevd"])
        self._run(["systemctl", "start", "systemd-udevd"])

        # And only then install snapd.
        self._run(["apt-get", "install", "snapd", "sudo", "--yes"])
        self._run(["systemctl", "start", "snapd"])

    def _setup_snapcraft(self):
        self._wait_for_network()
        super()._setup_snapcraft()

    def _ensure_container_running(self) -> None:
        # Sanity check - developer error if container not initialized.
        if self._container is None:
            raise RuntimeError("Attempted to use container before starting.")

        self._container.sync()
        if self._container.status.lower() != "running":
            raise errors.ProviderFileCopyError(
                provider_name=self._get_provider_name(),
                error_message=(
                    "Container is not running, the current state is: {!r}. "
                    "Ensure it has not been modified by external factors and try again"
                ).format(self._container.status),
            )
