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

"""Build environment provider support for snapcraft."""

import contextlib
import logging
import pathlib
from abc import ABC, abstractmethod
from typing import Generator, Tuple, Union

from craft_providers import Executor
from craft_providers.base import Base

logger = logging.getLogger(__name__)


class Provider(ABC):
    """Snapcraft's build environment provider."""

    def clean_project_environments(self, *, instance_name: str) -> None:
        """Clean the provider environment.

        :param instance_name: name of the instance to clean
        """
        # Nothing to do if provider is not installed.
        if not self.is_provider_installed():
            logger.debug(
                "Not cleaning environment because the provider is not installed."
            )
            return

        environment = self.create_environment(instance_name=instance_name)
        if environment.exists():
            environment.delete()

    @classmethod
    @abstractmethod
    def ensure_provider_is_available(cls) -> None:
        """Ensure provider is available, prompting the user to install it if required.

        :raises ProviderError: if provider is not available.
        """

    @classmethod
    def is_base_available(cls, base: str) -> Tuple[bool, Union[str, None]]:
        """Check if provider can provide an environment matching given base.

        :param base: Base to check.

        :returns: Tuple of bool indicating whether it is a match, with optional
                reason if not a match.
        """
        if base not in ["ubuntu:18.04", "ubuntu:20.04"]:
            return (
                False,
                f"Base {base!r} is not supported (must be 'ubuntu:18.04' or 'ubuntu:20.04')",
            )

        return True, None

    @classmethod
    @abstractmethod
    def is_provider_installed(cls) -> bool:
        """Check if provider is installed.

        :returns: True if installed.
        """

    @abstractmethod
    def create_environment(self, *, instance_name: str) -> Executor:
        """Create a bare environment for specified base.

        No initializing, launching, or cleaning up of the environment occurs.

        :param name: Name of the instance.
        """

    @abstractmethod
    @contextlib.contextmanager
    def launched_environment(
        self,
        *,
        project_name: str,
        project_path: pathlib.Path,
        base_configuration: Base,
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
