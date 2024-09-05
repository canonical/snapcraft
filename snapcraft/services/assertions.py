# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
#  Copyright 2024 Canonical Ltd.
#
#  This program is free software: you can redistribute it and/or modify it
#  under the terms of the GNU Lesser General Public License version 3, as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but WITHOUT
#  ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
#  SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Abstract service class for assertions."""

from __future__ import annotations

import abc
import json
from typing import Any

import craft_cli
import tabulate
from craft_application.services import base
from typing_extensions import override

from snapcraft import const, errors, models, store


class AssertionService(base.AppService):
    """Abstract service for interacting with assertions."""

    @override
    def setup(self) -> None:
        """Application-specific service setup."""
        self._store_client = store.StoreClientCLI()
        super().setup()

    @property
    @abc.abstractmethod
    def _assertion_type(self) -> str:
        """The pluralized name of the assertion type."""

    @abc.abstractmethod
    def _get_assertions(self, name: str | None = None) -> list[models.Assertion]:
        """Get assertions from the store.

        :param name: The name of the assertion to retrieve. If not provided, all
          assertions are retrieved.

        :returns: A list of assertions.
        """

    @abc.abstractmethod
    def _normalize_assertions(
        self, assertions: list[models.Assertion]
    ) -> tuple[list[str], list[list[Any]]]:
        """Convert a list of assertion models to a tuple of headers and data.

        :param assertions: A list of assertions to normalize.

        :returns: A tuple containing the headers and normalized assertions.
        """

    def list_assertions(self, *, output_format: str, name: str | None = None) -> None:
        """List assertions from the store.

        :param output_format: The output format to render.
        :param name: The name of the assertion to list. If not provided, all assertions
          are listed.

        :raises FeatureNotImplemented: If the output format is not supported.
        """
        assertions = self._get_assertions(name)

        if assertions:
            headers, normalized_assertions = self._normalize_assertions(assertions)
            match output_format:
                case const.OutputFormat.json:
                    json_assertions = {
                        self._assertion_type.lower(): [
                            {
                                header.lower(): value
                                for header, value in zip(headers, assertion)
                            }
                            for assertion in normalized_assertions
                        ]
                    }
                    craft_cli.emit.message(json.dumps(json_assertions, indent=4))
                case const.OutputFormat.table:
                    tabulated_sets = tabulate.tabulate(
                        normalized_assertions,
                        headers=headers,
                        tablefmt="plain",
                    )
                    craft_cli.emit.message(tabulated_sets)
                case _:
                    raise errors.FeatureNotImplemented(
                        msg=f"'--format {output_format}'",
                    )
        else:
            craft_cli.emit.message(f"No {self._assertion_type} found.")
