# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2019 Canonical Ltd
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

from snapcraft.internal.meta import errors


class TestErrorFormatting:

    scenarios = (
        (
            "MissingSnapcraftYamlKeysError",
            {
                "exception_class": errors.MissingSnapcraftYamlKeysError,
                "kwargs": {"keys": ["test-key1", "test-key2"]},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "Missing required key(s) in snapcraft.yaml: "
                    "'test-key1' and 'test-key2'. Either specify the missing "
                    "key(s), or use 'adopt-info' to get them from a part."
                ),
            },
        ),
        (
            "AdoptedPartMissingError",
            {
                "exception_class": errors.AdoptedPartMissingError,
                "kwargs": {"part": "test-part"},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "'adopt-info' refers to a part named 'test-part', but it is "
                    "not defined in the 'snapcraft.yaml' file."
                ),
            },
        ),
        (
            "AdoptedPartNotParsingInfo",
            {
                "exception_class": errors.AdoptedPartNotParsingInfo,
                "kwargs": {"part": "test-part"},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "'adopt-info' refers to part 'test-part', but that part is "
                    "lacking the 'parse-info' property."
                ),
            },
        ),
        (
            "AmbiguousPassthroughKeyError",
            {
                "exception_class": errors.AmbiguousPassthroughKeyError,
                "kwargs": {"keys": ["key1", "key2"]},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "The following keys are specified in their regular location "
                    "as well as in passthrough: 'key1' and 'key2'. "
                    "Remove duplicate keys."
                ),
            },
        ),
        (
            "InvalidAppCommandError",
            {
                "exception_class": errors.InvalidAppCommandError,
                "kwargs": {"command": "test-command", "app_name": "test-app"},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "The specified command 'test-command' defined in the app "
                    "'test-app' does not exist or is not executable.\n"
                    "Ensure that 'test-command' is installed with the correct path."
                ),
            },
        ),
        (
            "InvalidAppCommandNotFound",
            {
                "exception_class": errors.InvalidAppCommandNotFound,
                "kwargs": {"command": "test-command", "app_name": "test-app"},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "The specified command 'test-command' defined in the app "
                    "'test-app' does not exist.\n"
                    "Ensure that 'test-command' is installed with the correct path."
                ),
            },
        ),
        (
            "InvalidAppCommandNotExecutable",
            {
                "exception_class": errors.InvalidAppCommandNotExecutable,
                "kwargs": {"command": "test-command", "app_name": "test-app"},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "The specified command 'test-command' defined in the app "
                    "'test-app' is not executable."
                ),
            },
        ),
        (
            "InvalidAppCommandFormatError",
            {
                "exception_class": errors.InvalidAppCommandFormatError,
                "kwargs": {"command": "test-command", "app_name": "test-app"},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "The specified command 'test-command' defined in the app "
                    "'test-app' does not match the pattern expected by snapd.\n"
                    "The command must consist only of alphanumeric characters, spaces, "
                    "and the following special characters: / . _ # : $ -"
                ),
            },
        ),
        (
            "InvalidCommandChainError",
            {
                "exception_class": errors.InvalidCommandChainError,
                "kwargs": {"item": "test-chain", "app_name": "test-app"},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "The command-chain item 'test-chain' defined in the app 'test-app' "
                    "does not exist or is not executable.\n"
                    "Ensure that 'test-chain' is relative to the prime directory."
                ),
            },
        ),
        (
            "InvalidDesktopFileError",
            {
                "exception_class": errors.InvalidDesktopFileError,
                "kwargs": {"filename": "test-file", "message": "test-message"},
                "expected_message": (
                    "Failed to generate desktop file: "
                    "Invalid desktop file 'test-file': test-message."
                ),
            },
        ),
    )

    def test_error_formatting(self, exception_class, kwargs, expected_message):
        assert str(exception_class(**kwargs)) == expected_message


class TestSnapcraftException:

    scenarios = (
        (
            "GradeDevelRequiredError",
            {
                "exception_class": errors.GradeDevelRequiredError,
                "kwargs": dict(set_grade="stable"),
                "expected_brief": "Snap 'grade' was set to 'stable' but must be 'devel'.",
                "expected_resolution": "Set 'grade' to 'devel' or use a stable base for this snap.",
                "expected_details": None,
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "SystemUsernamesValidationError",
            {
                "exception_class": errors.SystemUsernamesValidationError,
                "kwargs": dict(name="user1", message="invalid ..."),
                "expected_brief": "Invalid system-usernames for 'user1': invalid ...",
                "expected_resolution": "Please configure system user according to documentation.",
                "expected_details": None,
                "expected_docs_url": "https://forum.snapcraft.io/t/system-usernames/13386",
                "expected_reportable": False,
            },
        ),
    )

    def test_snapcraft_exception_handling(
        self,
        exception_class,
        kwargs,
        expected_brief,
        expected_resolution,
        expected_details,
        expected_docs_url,
        expected_reportable,
    ):
        exception = exception_class(**kwargs)
        assert exception.get_brief() == expected_brief
        assert exception.get_resolution() == expected_resolution
        assert exception.get_details() == expected_details
        assert exception.get_docs_url() == expected_docs_url
        assert exception.get_reportable() == expected_reportable


def test_PackageRepositoriesValidationError():
    error = errors.PackageRepositoryValidationError(
        url="http://foo", brief="some error", resolution="some way to fix"
    )

    assert (
        error.get_brief() == "Invalid package-repository for 'http://foo': some error"
    )
    assert error.get_resolution() == "some way to fix"
    assert error.get_details() is None
    assert error.get_docs_url() == "https://snapcraft.io/docs/package-repositories"
    assert error.get_reportable() is False


def test_PackageRepositoriesValidationError_no_resolution():
    error = errors.PackageRepositoryValidationError(
        url="http://foo", brief="some error"
    )

    assert (
        error.get_resolution()
        == "You can verify package repository configuration according to the referenced documentation."
    )
