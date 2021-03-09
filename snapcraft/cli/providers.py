# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021 Canonical Ltd.
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
import logging
import os
import pathlib
import subprocess
import tempfile
import textwrap
from typing import List, Optional, Union

from craft_providers import Executor
from craft_providers.images import BuilddImage, BuilddImageAlias
from craft_providers.lxd import LXDInstance, LXDProvider
from craft_providers.multipass import MultipassInstance, MultipassProvider

from snapcraft.internal.repo import snaps

from . import echo
from ._options import ProviderOptions

logger = logging.getLogger(__name__)


def base_to_alias(base: str):
    if base == "core":
        return BuilddImageAlias.XENIAL
    elif base == "core18":
        return BuilddImageAlias.BIONIC
    elif base == "core20":
        return BuilddImageAlias.FOCAL

    return BuilddImageAlias.FOCAL


class SnapcraftBuilddImage(BuilddImage):
    def __init__(
        self,
        *,
        base: str,
        http_proxy: Optional[str] = None,
        https_proxy: Optional[str] = None,
        compatibility_tag: str = "snapcraft-buildd-image-v0",
        hostname: str = "snapcraft-buildd-instance",
    ):
        alias = base_to_alias(base)

        super().__init__(
            alias=alias,
            compatibility_tag=compatibility_tag,
            hostname=hostname,
            http_proxy=http_proxy,
            https_proxy=https_proxy,
        )

    def _get_host_snap_version(self) -> Optional[str]:
        """Get snapcraft version on host (only if snap)."""
        if os.getenv("SNAP") is None or os.getenv("SNAP_NAME") != "snapcraft":
            return None

        return os.getenv("SNAP_VERSION", None)

    def _get_instance_snap_version(self, *, executor: Executor) -> Optional[str]:
        """Get snapcraft version inside instance."""
        try:
            return (
                executor.execute_run(
                    ["snapcraft", "version"], check=True, capture_output=True,
                )
                .stdout.decode()
                .strip()
                .replace("snapcraft, version ", "")
            )
        except subprocess.CalledProcessError:
            return None

    def _inject_snap(self, *, executor: Executor) -> None:
        """Inject snap from host and write to /tmp/snapcraft.snap."""
        snap_iter = snaps._get_local_snap_file_iter("snapcraft", chunk_size=1024 * 1024)
        with tempfile.NamedTemporaryFile() as temp_file:
            for chunk in snap_iter:
                temp_file.write(chunk)

            executor.push(
                source=pathlib.Path(temp_file.name),
                destination=pathlib.Path("/tmp/snapcraft.snap"),
            )

    def _fetch_snap(self, *, executor: Executor) -> None:
        """Fetch snap from store and write to /tmp/snapcraft.snap."""
        executor.execute_run(
            [
                "snap",
                "download",
                "snapcraft",
                "--basename=snapcraft",
                "--target-directory=/tmp",
            ],
            check=True,
        )

    def _install_snap(self, *, executor: Executor) -> None:
        host_version = self._get_host_snap_version()
        instance_version = self._get_instance_snap_version(executor=executor)
        logger.debug(
            "host snap version: %s -- instance snap version: %s",
            host_version,
            instance_version,
        )

        if host_version is None and instance_version is None:
            # Not running as snap, fetch from store.
            self._fetch_snap(executor=executor)
        elif host_version is not None and host_version != instance_version:
            # Running as snap, inject if installed version doesn't match.
            self._inject_snap(executor=executor)
        else:
            # Nothing to do (already installed).
            return

        executor.execute_run(
            ["snap", "install", "/tmp/snapcraft.snap", "--classic", "--dangerous"],
            check=True,
            capture_output=True,
        )

    def setup(self, *, executor: Executor) -> None:
        super().setup(executor=executor)

        executor.execute_run(
            [
                "apt-get",
                "install",
                "-y",
                "apt-transport-https",
                "dirmngr",
                "gnupg",
                "sudo",
            ],
            check=True,
        )

        self._install_snap(executor=executor)

        proc = executor.execute_run(
            ["snapcraft", "version"], check=True, stdout=subprocess.PIPE
        )
        version = proc.stdout.decode().strip()
        logger.debug("Running: %s", version)

        logger.debug("Installing Snapcraft dependencies...")
        executor.create_file(
            destination=pathlib.Path("/root/.bashrc"),
            content='#!/bin/bash\nexport PS1="\\h \\$(/usr/bin/_snapcraft_prompt)# "\n'.encode(),
            file_mode="0644",
        )

        executor.create_file(
            destination=pathlib.Path("/usr/bin/_snapcraft_prompt"),
            content=textwrap.dedent(
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
            ).encode(),
            file_mode="0755",
        )


def get_instance_name(*, project_name: str) -> str:
    return f"snapcraft-{project_name}"


def request_user_install(provider: str) -> None:
    if echo.is_tty_connected() and echo.confirm(
        f"Support for {provider} needs to be set up. Would you like to do it now?"
    ):
        return

    raise RuntimeError(f"Please install {provider}.")


class InstanceManager(abc.ABC):
    def __init__(
        self,
        instance: Union[LXDInstance, MultipassInstance],
        provider_options: ProviderOptions,
    ) -> None:
        self.environment = dict(
            SNAPCRAFT_BUILD_ENVIRONMENT="managed-host", HOME="/root",
        )

        if provider_options.http_proxy:
            self.environment["http_proxy"] = provider_options.http_proxy

        if provider_options.https_proxy:
            self.environment["https_proxy"] = provider_options.https_proxy

        self.instance = instance
        self.provider_options = provider_options

    @abc.abstractmethod
    def clean(self) -> None:
        """Clean instance."""

    def clean_parts(self, parts: List[str]) -> None:
        command = ["snapcraft", "clean", *parts]
        self.instance.execute_run(command, check=True, env=self.environment)

    def execute_step(self, step: str) -> None:
        command = ["snapcraft", step]
        self.instance.execute_run(command, check=True, env=self.environment)

    def configure_instance(self, *, project_dir: pathlib.Path):
        """Final configuration steps for instances prior to building."""
        self.mount_project(project_dir)

        if self.provider_options.bind_ssh:
            self.mount_ssh(pathlib.Path.home() / ".ssh")

        if self.provider_options.add_ca_certificates is not None:
            self.install_certificates(
                pathlib.Path(self.provider_options.add_ca_certificates)
            )

    def install_certificates(self, certs_path: pathlib.Path) -> None:
        if certs_path.is_file():
            certificate_files = [certs_path]
        elif certs_path.is_dir():
            certificate_files = [x for x in certs_path.iterdir() if x.is_file()]
        else:
            raise RuntimeError(f"Unable to read CA certificates: {certs_path!r}")

        for certificate_file in sorted(certificate_files):
            logger.info(f"Installing CA certificate: {certificate_file}")
            content = certificate_file.read_bytes()

            self.instance.create_file(
                destination=pathlib.Path("/usr/local/share/ca-certificates")
                / certificate_file.name,
                content=content,
                file_mode="0644",
            )

        if certificate_files:
            self.instance.execute_run(
                ["update-ca-certificates"], check=True, env=self.environment
            )

    def mount_project(self, project_dir: pathlib.Path) -> None:
        self.instance.mount(
            host_source=project_dir, target=pathlib.Path("/root/project")
        )

    def mount_ssh(self, ssh_dir: pathlib.Path) -> None:
        self.instance.mount(host_source=ssh_dir, target=pathlib.Path("/root/.ssh"))

    def mount_prime(self, prime_dir: pathlib.Path) -> None:
        os.makedirs(prime_dir, exist_ok=True)

        self.instance.mount(host_source=prime_dir, target=pathlib.Path("/root/prime"))
        self.instance.execute_run(
            ["snapcraft", "clean", "--unprime"], check=True, env=self.environment
        )

    def pack_project(self, output: Optional[str] = None) -> None:
        command = ["snapcraft", "snap"]
        if output:
            command.extend(["--output", output])

        self.instance.execute_run(command, check=True, env=self.environment)

    def shell(self) -> None:
        self.instance.execute_run(["bash"], check=True, env=self.environment)


class LXDInstanceManager(InstanceManager):
    def __init__(
        self,
        *,
        instance: LXDInstance,
        provider: LXDProvider,
        provider_options: ProviderOptions,
    ) -> None:
        super().__init__(instance=instance, provider_options=provider_options)

        self.provider = provider

    def clean(self) -> None:
        if self.instance.exists():
            self.instance.delete(force=True)

    @classmethod
    def setup(cls, *, name: str, base: str, provider_options: ProviderOptions):
        if provider_options.lxd_image_remote is not None:
            image_remote = provider_options.lxd_image_remote
        else:
            image_remote = "buildd"

        if provider_options.lxd_image_name is not None:
            image_name = provider_options.lxd_image_name
        else:
            image_name = base

        image_configuration = SnapcraftBuilddImage(
            base=base,
            hostname=name,
            http_proxy=provider_options.http_proxy,
            https_proxy=provider_options.https_proxy,
        )
        provider = LXDProvider()

        if not provider.is_installed():
            request_user_install("LXD")
            provider.install()

        # Ensure project exists (does nothing if already present).
        provider.create_project(
            name=provider_options.lxd_project, remote=provider_options.lxd_remote
        )

        # Ensure buildd remote is configured, if used.
        if image_remote == "buildd" and not provider.is_image_remote_installed(
            name="buildd"
        ):
            logger.debug("Adding remote for buildd images...")
            provider.create_image_remote(
                name="buildd",
                addr="https://cloud-images.ubuntu.com/buildd/releases",
                protocol="simplestreams",
            )

        logger.debug("Creating build environment...")
        instance = provider.create_instance(
            image_configuration=image_configuration,
            image_name=image_name,
            image_remote=image_remote,
            name=name,
            auto_clean=True,
            project=provider_options.lxd_project,
            remote=provider_options.lxd_remote,
            use_snapshots=provider_options.enable_snapshots,
        )

        return cls(
            instance=instance, provider=provider, provider_options=provider_options
        )

    @classmethod
    def setup_existing(cls, *, name: str, provider_options: ProviderOptions):
        instance = LXDInstance(
            name=name,
            project=provider_options.lxd_project,
            remote=provider_options.lxd_remote,
        )
        provider = LXDProvider()

        return cls(
            instance=instance, provider=provider, provider_options=provider_options
        )


class MultipassInstanceManager(InstanceManager):
    def __init__(
        self,
        *,
        instance: LXDInstance,
        provider: LXDProvider,
        provider_options: ProviderOptions,
    ) -> None:
        super().__init__(instance=instance, provider_options=provider_options)

        self.provider = provider

    def clean(self) -> None:
        if self.instance.exists():
            self.instance.delete(purge=True)

    @classmethod
    def setup(cls, *, name: str, base: str, provider_options: ProviderOptions):
        image_configuration = SnapcraftBuilddImage(base=base, hostname=name)
        provider = MultipassProvider()

        if not provider.is_installed():
            request_user_install("Multipass")
            provider.install()

        logger.debug("Creating build environment...")
        instance = provider.create_instance(
            image_configuration=image_configuration,
            image_name=f"snapcraft:{base}",
            name=name,
            auto_clean=True,
        )

        return cls(
            instance=instance, provider=provider, provider_options=provider_options
        )

    @classmethod
    def setup_existing(cls, *, name: str, provider_options: ProviderOptions):
        instance = MultipassInstance(name=name)
        provider = MultipassProvider()

        return cls(
            instance=instance, provider=provider, provider_options=provider_options
        )


def setup_instance_manager(
    *,
    base: str,
    project_dir: pathlib.Path,
    project_name: str,
    provider_options: ProviderOptions,
) -> InstanceManager:
    name = get_instance_name(project_name=project_name)

    if provider_options.provider == "lxd":
        manager = LXDInstanceManager.setup(
            name=name, base=base, provider_options=provider_options
        )
    elif provider_options.provider == "multipass":
        manager = MultipassInstanceManager.setup(
            name=name, base=base, provider_options=provider_options
        )
    else:
        raise RuntimeError("unsupported provider")

    manager.configure_instance(project_dir=project_dir)
    return manager


def setup_existing_instance_manager(
    *, project_name: str, provider_options: ProviderOptions
):
    name = get_instance_name(project_name=project_name)

    if provider_options.provider == "lxd":
        manager = LXDInstanceManager.setup_existing(
            name=name, provider_options=provider_options
        )
    elif provider_options.provider == "multipass":
        manager = MultipassInstanceManager.setup_existing(
            name=name, provider_options=provider_options
        )
    else:
        raise RuntimeError("unsupported provider")

    return manager
