# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021-2022 Canonical Ltd.
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

"""Multipass build environment provider for Snapcraft."""

import contextlib
import logging
import pathlib
from typing import Generator, Optional

from craft_cli import emit
from craft_providers import Executor, ProviderError, bases, multipass
from craft_providers.multipass.errors import MultipassError

from snapcraft import utils

from ._buildd import BASE_TO_BUILDD_IMAGE_ALIAS, SnapcraftBuilddBaseConfiguration
from ._provider import Provider
from .providers import get_command_environment, get_instance_name

logger = logging.getLogger(__name__)


class MultipassProvider(Provider):
    """Multipass build environment provider.

    :param multipass: Optional Multipass client to use.
    """

    def __init__(
        self,
        instance: multipass.Multipass = multipass.Multipass(),
    ) -> None:
        self.multipass = instance

    @classmethod
    def ensure_provider_is_available(cls) -> None:
        """Ensure provider is available, prompting the user to install it if required.

        :raises ProviderError: if provider is not available.
        """
        if not multipass.is_installed():
            with emit.pause():
                confirmation = utils.confirm_with_user(
                    "Multipass is required, but not installed. Do you wish to install Multipass "
                    "and configure it with the defaults?",
                    default=False,
                )
            if confirmation:
                try:
                    multipass.install()
                except multipass.MultipassInstallationError as error:
                    raise ProviderError(
                        "Failed to install Multipass. Visit https://multipass.run/ for "
                        "instructions on installing Multipass for your operating system.",
                    ) from error
            else:
                raise ProviderError(
                    "Multipass is required, but not installed. Visit https://multipass.run/ for "
                    "instructions on installing Multipass for your operating system.",
                )

        try:
            multipass.ensure_multipass_is_ready()
        except multipass.MultipassError as error:
            raise ProviderError(str(error)) from error

    @classmethod
    def is_provider_available(cls) -> bool:
        """Check if provider is installed and available for use.

        :returns: True if installed.
        """
        return multipass.is_installed()

    def create_environment(self, *, instance_name: str) -> Executor:
        """Create a bare environment for specified base.

        No initializing, launching, or cleaning up of the environment occurs.

        :param name: Name of the instance.
        """
        return multipass.MultipassInstance(name=instance_name)

    @contextlib.contextmanager
    def launched_environment(
        self,
        *,
        project_name: str,
        project_path: pathlib.Path,
        base: str,
        bind_ssh: bool,
        build_on: str,
        build_for: str,
        http_proxy: Optional[str] = None,
        https_proxy: Optional[str] = None,
    ) -> Generator[Executor, None, None]:
        """Launch environment for specified base.

        The environment is launched and configured using the base configuration.
        Upon exit, drives are unmounted and the environment is stopped.

        :param project_name: Name of the project.
        :param project_path: Path to project.
        :param base: Base to create.
        :param bind_ssh: If true, mount the host's ssh directory in the environment.
        :param build_on: host architecture
        :param build_for: target architecture
        """
        alias = BASE_TO_BUILDD_IMAGE_ALIAS[base]

        instance_name = get_instance_name(
            project_name=project_name,
            project_path=project_path,
            build_on=build_on,
            build_for=build_for,
        )

        environment = get_command_environment(
            http_proxy=http_proxy, https_proxy=https_proxy
        )
        base_configuration = SnapcraftBuilddBaseConfiguration(
            alias=alias,  # type: ignore
            environment=environment,
            hostname=instance_name,
        )

        try:
            instance = multipass.launch(
                name=instance_name,
                base_configuration=base_configuration,
                image_name=f"snapcraft:{base}",
                cpus=2,
                disk_gb=64,
                mem_gb=2,
                auto_clean=True,
            )
        except (bases.BaseConfigurationError, MultipassError) as error:
            raise ProviderError(str(error)) from error

        try:
            # Mount project.
            instance.mount(
                host_source=project_path,
                target=utils.get_managed_environment_project_path(),
            )

            # Mount ssh directory.
            if bind_ssh:
                instance.mount(
                    host_source=pathlib.Path.home() / ".ssh",
                    target=utils.get_managed_environment_home_path() / ".ssh",
                )

        except MultipassError as error:
            raise ProviderError(str(error)) from error

        try:
            yield instance
        finally:
            # Ensure to unmount everything and stop instance upon completion.
            try:
                instance.unmount_all()
                instance.stop()
            except MultipassError as error:
                raise ProviderError(str(error)) from error
