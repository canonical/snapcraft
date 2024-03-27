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

"""Snapcraft lifecycle commands."""

import argparse
from typing import Any

import craft_application.commands
from craft_cli import emit
from typing_extensions import override

# pylint: disable=too-many-ancestors


class SnapCommand(craft_application.commands.lifecycle.PackCommand):
    """Deprecated legacy command to pack the final snap payload."""

    name = "snap"
    hidden = True

    @override
    def _run(
        self,
        parsed_args: argparse.Namespace,
        step_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        emit.progress(
            "Warning: the 'snap' command is deprecated and will be removed "
            "in a future release of Snapcraft. Please use 'pack' instead.",
            permanent=True,
        )

        super()._run(parsed_args, step_name, **kwargs)
