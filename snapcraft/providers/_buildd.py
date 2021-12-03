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

"""Buildd-related helpers for Snapcraft."""

import enum
import sys
from typing import Optional

from craft_providers import Executor, bases
from craft_providers.actions import snap_installer

from snapcraft import utils


# XXX: update in craft-providers
class _BuilddBaseAlias(enum.Enum):
    """Mappings for supported buildd images."""

    JAMMY = "22.04"


BASE_TO_BUILDD_IMAGE_ALIAS = {
    # "core22": bases.BuilddBaseAlias.JAMMY,
    "core22": _BuilddBaseAlias.JAMMY,
}


class SnapcraftBuilddBaseConfiguration(bases.BuilddBase):
    """Base configuration for Snapcraft.

    :cvar compatibility_tag: Tag/Version for variant of build configuration and
        setup.  Any change to this version would indicate that prior [versioned]
        instances are incompatible and must be cleaned.  As such, any new value
        should be unique to old values (e.g. incrementing).  Snapcraft extends
        the buildd tag to include its own version indicator (.0) and namespace
        ("snapcraft").
    """

    compatibility_tag: str = f"snapcraft-{bases.BuilddBase.compatibility_tag}.0"

    @staticmethod
    def _setup_snapcraft(*, executor: Executor) -> None:
        """Install Snapcraft in target environment.

        On Linux, the default behavior is to inject the host snap into the target
        environment.

        On other platforms, the Snapcraft snap is installed from the Snap Store.

        When installing the snap from the Store, we check if the user specifies a
        channel, using SNAPCRAFT_INSTALL_SNAP_CHANNEL=<channel>.  If unspecified,
        we use the "stable" channel on the default track.

        On Linux, the user may specify this environment variable to force Snapcraft
        to install the snap from the Store rather than inject the host snap.

        :raises BaseConfigurationError: on error.
        """
        snap_channel = utils.get_managed_environment_snap_channel()
        if snap_channel is None and sys.platform != "linux":
            snap_channel = "stable"

        if snap_channel:
            try:
                snap_installer.install_from_store(
                    executor=executor,
                    snap_name="snapcraft",
                    channel=snap_channel,
                    classic=True,
                )
            except snap_installer.SnapInstallationError as error:
                raise bases.BaseConfigurationError(
                    brief="Failed to install snapcraft snap from store channel "
                    f"{snap_channel!r} into target environment."
                ) from error
        else:
            try:
                snap_installer.inject_from_host(
                    executor=executor, snap_name="snapcraft", classic=True
                )
            except snap_installer.SnapInstallationError as error:
                raise bases.BaseConfigurationError(
                    brief="Failed to inject host snapcraft snap into target environment."
                ) from error

    def setup(
        self,
        *,
        executor: Executor,
        retry_wait: float = 0.25,
        timeout: Optional[float] = None,
    ) -> None:
        """Prepare base instance for use by the application.

        :param executor: Executor for target container.
        :param retry_wait: Duration to sleep() between status checks (if
            required).
        :param timeout: Timeout in seconds.

        :raises BaseCompatibilityError: if instance is incompatible.
        :raises BaseConfigurationError: on other unexpected error.
        """
        super().setup(executor=executor, retry_wait=retry_wait, timeout=timeout)
        self._setup_snapcraft(executor=executor)
