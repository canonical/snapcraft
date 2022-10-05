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
from typing import Generator

from craft_providers import Executor, ProviderError, base, bases, multipass
from craft_providers.multipass.errors import MultipassError

from ._provider import Provider

logger = logging.getLogger(__name__)


PROVIDER_BASE_TO_MULTIPASS_BASE = {
    bases.BuilddBaseAlias.BIONIC.value: "snapcraft:18.04",
    bases.BuilddBaseAlias.FOCAL.value: "snapcraft:20.04",
    bases.BuilddBaseAlias.JAMMY.value: "snapcraft:22.04",
}


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
            try:
                multipass.install()
            except multipass.MultipassInstallationError as error:
                raise ProviderError(
                    "Failed to install Multipass. Visit https://multipass.run/ for "
                    "instructions on installing Multipass for your operating system.",
                ) from error

        try:
            multipass.ensure_multipass_is_ready()
        except multipass.MultipassError as error:
            raise ProviderError(str(error)) from error

    @classmethod
    def is_provider_installed(cls) -> bool:
        """Check if provider is installed.

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
        base_configuration: base.Base,
        build_base: str,
        instance_name: str,
    ) -> Generator[Executor, None, None]:
        """Configure and launch environment for specified base.

        When this method loses context, all directories are unmounted and the
        environment is stopped. For more control of environment setup and teardown,
        use `create_environment()` instead.

        :param project_name: Name of project.
        :param project_path: Path to project.
        :param base_configuration: Base configuration to apply to instance.
        :param build_base: Base to build from.
        :param instance_name: Name of the instance to launch.
        """
        try:
            instance = multipass.launch(
                name=instance_name,
                base_configuration=base_configuration,
                image_name=PROVIDER_BASE_TO_MULTIPASS_BASE[build_base],
                cpus=2,
                disk_gb=64,
                mem_gb=2,
                auto_clean=True,
            )
        except (bases.BaseConfigurationError, MultipassError) as error:
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
