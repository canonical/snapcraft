# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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
"""Snapcraft Lifecycle Service."""

import pathlib

import craft_cli
import platformdirs
from craft_application import launchpad
from craft_application.services import remotebuild
from typing_extensions import override


class RemoteBuild(remotebuild.RemoteBuildService):
    """Snapcraft remote build service."""

    RecipeClass = launchpad.models.SnapRecipe

    __credentials_filepath: pathlib.Path | None = None

    @property
    @override
    def credentials_filepath(self) -> pathlib.Path:
        """The filepath to the Launchpad credentials.

        The legacy credentials are only loaded when they exist and the new credentials
        do not exist. If the legacy credentials are loaded, emit a deprecation warning.
        """
        # return early so the deprecation notice is emitted only once
        if self.__credentials_filepath:
            return self.__credentials_filepath

        credentials_filepath = super().credentials_filepath
        legacy_credentials_filepath = (
            platformdirs.user_data_path("snapcraft") / "provider/launchpad/credentials"
        )

        if not credentials_filepath.exists() and legacy_credentials_filepath.exists():
            craft_cli.emit.progress(
                f"Warning: Using launchpad credentials from deprecated location {str(legacy_credentials_filepath)!r}.\n"
                f"Credentials should be migrated to {str(credentials_filepath)!r}.",
                permanent=True,
            )
            self.__credentials_filepath = legacy_credentials_filepath
        else:
            self.__credentials_filepath = credentials_filepath

        return self.__credentials_filepath
