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

"""LXD build environment provider support for Snapcraft."""

import contextlib
import logging
import os
import pathlib
from typing import Generator, List

from craft_providers import Executor, bases, lxd

from snapcraft.utils import confirm_with_user, get_managed_environment_project_path

from ._buildd import BASE_TO_BUILDD_IMAGE_ALIAS, SnapcraftBuilddBaseConfiguration
from ._provider import Provider, ProviderError

logger = logging.getLogger(__name__)

_BASE_IMAGE = {"core22": "ubuntu:22.04"}


class LXDProvider(Provider):
    """LXD build environment provider.

    :param lxc: Optional lxc client to use.
    :param lxd_project: LXD project to use (default is snapcraft).
    :param lxd_remote: LXD remote to use (default is local).
    """

    def __init__(
        self,
        *,
        lxc: lxd.LXC = lxd.LXC(),
        lxd_project: str = "snapcraft",
        lxd_remote: str = "local",
    ) -> None:
        self.lxc = lxc
        self.lxd_project = lxd_project
        self.lxd_remote = lxd_remote

    def clean_project_environments(
        self, *, project_name: str, project_path: pathlib.Path
    ) -> List[str]:
        """Clean up any build environments created for project.

        :param project_name: Name of project.

        :returns: List of containers deleted.
        """
        deleted: List[str] = []

        # Nothing to do if provider is not installed.
        if not self.is_provider_available():
            return deleted

        instance_name = self.get_instance_name(
            project_name=project_name,
            project_path=project_path,
        )

        try:
            names = self.lxc.list_names(
                project=self.lxd_project, remote=self.lxd_remote
            )
        except lxd.LXDError as error:
            raise ProviderError(str(error)) from error

        for name in names:
            if name == instance_name:
                logger.debug("Deleting container %r.", name)
                try:
                    self.lxc.delete(
                        instance_name=name,
                        force=True,
                        project=self.lxd_project,
                        remote=self.lxd_remote,
                    )
                except lxd.LXDError as error:
                    raise ProviderError(str(error)) from error
                deleted.append(name)
            else:
                logger.debug("Not deleting container %r.", name)

        return deleted

    @classmethod
    def ensure_provider_is_available(cls) -> None:
        """Ensure provider is available, prompting the user to install it if required.

        :raises ProviderError: if provider is not available.
        """
        if not lxd.is_installed():
            if confirm_with_user(
                "LXD is required, but not installed. Do you wish to install LXD "
                "and configure it with the defaults?",
                default=False,
            ):
                try:
                    lxd.install()
                except lxd.LXDInstallationError as error:
                    raise ProviderError(
                        "Failed to install LXD. Visit https://snapcraft.io/lxd for "
                        "instructions on how to install the LXD snap for your distribution",
                    ) from error
            else:
                raise ProviderError(
                    "LXD is required, but not installed. Visit https://snapcraft.io/lxd "
                    "for instructions on how to install the LXD snap for your distribution",
                )

        try:
            lxd.ensure_lxd_is_ready()
        except lxd.LXDError as error:
            raise ProviderError(str(error)) from error

    @classmethod
    def is_provider_available(cls) -> bool:
        """Check if provider is installed and available for use.

        :returns: True if installed.
        """
        return lxd.is_installed()

    @contextlib.contextmanager
    def launched_environment(
        self,
        *,
        project_name: str,
        project_path: pathlib.Path,
        base: str,
    ) -> Generator[Executor, None, None]:
        """Launch environment for specified base.

        :param project_name: Name of project.
        :param project_path: Path to project.
        :param base: Base to create.
        """
        alias = BASE_TO_BUILDD_IMAGE_ALIAS[base]

        instance_name = self.get_instance_name(
            project_name=project_name,
            project_path=project_path,
        )

        base_image = _BASE_IMAGE[base]
        if ":" in base_image:
            image_remote, image_name = base_image.split(":", 1)
        else:
            try:
                image_remote = lxd.configure_buildd_image_remote()
                image_name = base_image
            except lxd.LXDError as error:
                raise ProviderError(str(error)) from error

        environment = self.get_command_environment()

        base_configuration = SnapcraftBuilddBaseConfiguration(
            alias=alias,  # type: ignore
            environment=environment,
            hostname=instance_name,
        )

        try:
            instance = lxd.launch(
                name=instance_name,
                base_configuration=base_configuration,
                image_name=image_name,
                image_remote=image_remote,
                auto_clean=True,
                auto_create_project=True,
                map_user_uid=True,
                uid=os.stat(project_path).st_uid,
                use_snapshots=True,
                project=self.lxd_project,
                remote=self.lxd_remote,
            )
        except (bases.BaseConfigurationError, lxd.LXDError) as error:
            raise ProviderError(str(error)) from error

        # Mount project.
        instance.mount(
            host_source=project_path, target=get_managed_environment_project_path()
        )

        try:
            yield instance
        finally:
            # Ensure to unmount everything and stop instance upon completion.
            try:
                instance.unmount_all()
                instance.stop()
            except lxd.LXDError as error:
                raise ProviderError(str(error)) from error
