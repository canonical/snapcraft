# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

"""Main Snapcraft Application."""

from __future__ import annotations

import argparse
import logging
from typing import Any

import craft_cli
import craft_parts
from craft_application import Application, AppMetadata, util
from craft_cli import emit
from overrides import override

from snapcraft import services, cli, utils, projects, errors
from snapcraft.commands import app_lifecycle
from snapcraft.parts import yaml_utils

APP_METADATA = AppMetadata(
    name="snapcraft",
    summary="A tool to create OCI images",
    ProjectClass=projects.SnapcraftProject,
    source_ignore_patterns=["*.snap"],
)


class Snapcraft(Application):
    """Snapcraft application definition."""

    project: projects.SnapcraftProject

    @override
    def configure(self, global_args: dict[str, Any]) -> None:
        self._global_args = global_args  # TODO: Don't do this!
    @override
    def _extra_yaml_transform(self, yaml_data: dict[str, Any]) -> dict[str, Any]:
        # TODO: Core22+ project model that raises appropriate exceptions for legacy bases.

        host_arch = utils.get_host_architecture()
        # TODO: Get the build-for a better way
        # Apply craft-grammar (this runs in the manager and in managed mode)
        yaml_data = yaml_utils.apply_yaml(
            yaml_data,
            host_arch,
            self._global_args.get("build-for", host_arch)
        )

        # TODO: Fill info from adopt-info, etc.

        return yaml_data

    @property
    def command_groups(self) -> list[craft_cli.CommandGroup]:
        """Return command groups."""
        # Dirty hack to avoid craft-application's default commands
        return self._command_groups

    @override
    def _configure_services(self, platform: str | None, build_for: str | None) -> None:
        if build_for is None:
            build_for = util.get_host_architecture()

        self.services.set_kwargs("image", work_dir=self._work_dir, build_for=build_for)
        self.services.set_kwargs(
            "package",
            platform=platform,
            build_for=build_for,
        )
        super()._configure_services(platform, build_for)


def main() -> int:
    """Run craft-application based charmcraft with classic fallback."""

    for lib_name in ("craft_parts", "craft_providers", "craft_store", "snapcraft.remote"):
        logger = logging.getLogger(lib_name)
        logger.setLevel(logging.DEBUG)

    charmcraft_services = services.SnapcraftServiceFactory(app=APP_METADATA)

    print("\n\n\nWELCOME TO THE FANCY NEW CRAFT-APPLICATION BASED SNAPCRAFT!\n\n\n")

    app = Snapcraft(
        app=APP_METADATA,
        services=charmcraft_services
    )

    # for group in cli.COMMAND_GROUPS:
    #     app.add_command_group(
    #         group.name, group.commands
    #     )

    app.add_command_group(
        "Lifecycle",
        [*app_lifecycle.get_lifecycle_command_group().commands],
    )


    try:
        return app.run()
    except errors.ClassicFallback:
        raise Exception("Classic explicitly disabled for now.")
        emit.debug("Falling back to classic")
        return cli.run()