# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from tests import unit

from testtools.matchers import Equals

from snapcraft.project import errors


class ErrorFormattingTest(unit.TestCase):

    scenarios = [
        (
            "MissingSnapcraftYamlError",
            {
                "exception": errors.MissingSnapcraftYamlError,
                "kwargs": {"snapcraft_yaml_file_path": ".snapcraft.yaml"},
                "expected_message": (
                    "Could not find .snapcraft.yaml. Are you sure you are "
                    "in the right directory?\n"
                    "To start a new project, use `snapcraft init`"
                ),
            },
        ),
        (
            "YamlValidationError",
            {
                "exception": errors.YamlValidationError,
                "kwargs": {"source": ".snapcraft.yaml", "message": "error"},
                "expected_message": ("Issues while validating .snapcraft.yaml: error"),
            },
        ),
        (
            "DuplicateSnapcraftYamlError",
            {
                "exception": errors.DuplicateSnapcraftYamlError,
                "kwargs": {
                    "snapcraft_yaml_file_path": ".snapcraft.yaml",
                    "other_snapcraft_yaml_file_path": "snapcraft.yaml",
                },
                "expected_message": (
                    "Found a '.snapcraft.yaml' and a 'snapcraft.yaml'.\n"
                    "Please remove one and try again."
                ),
            },
        ),
    ]

    def test_error_formatting(self):
        self.assertThat(
            str(self.exception(**self.kwargs)), Equals(self.expected_message)
        )
