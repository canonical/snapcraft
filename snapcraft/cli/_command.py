# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import logging
import os
import sys

import click

from snapcraft.internal import common, deprecations, errors

from ._options import get_build_provider, get_project


def run_legacy_snapcraft(argv=sys.argv[1:]) -> None:
    deprecations.handle_deprecation_notice("dn13")

    if not common.is_snap():
        raise errors.SnapcraftEnvironmentError(
            "Legacy mode not supported in this installation. "
            "Install snapcraft from https://snapcraft.io/snapcraft and try again."
        )

    legacy_python = os.path.join(
        common.get_legacy_snapcraft_dir(), "usr", "bin", "python3"
    )
    legacy_snapcraft = os.path.join(
        common.get_legacy_snapcraft_dir(), "bin", "snapcraft"
    )

    cmd = [legacy_python, legacy_snapcraft] + argv
    logging.debug("Running legacy snapcraft with: {}".format(cmd))
    os.execv(legacy_python, cmd)


class SnapcraftProjectCommand(click.Command):
    """
    Project related command that can fetch properties from the legacy tree.
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    def _is_legacy(self) -> bool:
        try:
            build_provider = get_build_provider(skip_sanity_checks=True)
            is_managed_host = build_provider == "managed-host"
            project = get_project(is_managed_host=is_managed_host)
        except Exception:
            # If we cannot load the project, we assume we are in non legacy
            # where the error should be properly handled.
            return False

        # Not in a project.
        if project is None:
            return False

        return project.is_legacy_project()

    def get_help(self, ctx):
        if self._is_legacy():
            run_legacy_snapcraft(argv=["help", self.name])
        else:
            return super().get_help(ctx)

    def invoke(self, ctx):
        if self._is_legacy():
            run_legacy_snapcraft()
        else:
            return super().invoke(ctx)
