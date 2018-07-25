# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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
from unittest import mock

import fixtures
from testtools.matchers import Contains, Equals

from snapcraft import storeapi
from snapcraft.cli import assertions
from tests import unit
from . import StoreCommandsBaseTestCase


class CollaborateBaseTestCase(StoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()
        patcher = mock.patch("subprocess.Popen")
        self.popen_mock = patcher.start()
        process_mock = mock.Mock()
        process_mock.returncode = 0
        process_mock.communicate.return_value = [b"foo", b""]
        self.popen_mock.return_value = process_mock
        self.addCleanup(patcher.stop)
        patcher = mock.patch("snapcraft.cli.assertions._edit_developers")
        self.edit_developers_mock = patcher.start()
        self.edit_developers_mock.return_value = [
            {"developer-id": "dummy-id", "since": "2015-07-19 19:30:00"}
        ]
        self.addCleanup(patcher.stop)
        self.client.login("dummy", "test correct password")


class CollaboratorsCommandTestCase(CollaborateBaseTestCase):

    scenarios = [
        ("edit-collaborators", {"command_name": "edit-collaborators"}),
        ("collaborators alias", {"command_name": "collaborators"}),
    ]

    def test_collaborators_without_snap_name_must_error(self):
        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_collaborators_update_developers(self):
        result = self.run_command(
            [self.command_name, "test-snap-with-dev", "--key-name", "key-name"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.popen_mock.assert_called_with(
            ["snap", "sign", "-k", "key-name"], stderr=-1, stdin=-1, stdout=-1
        )

    def test_collaborators_add_developers(self):
        result = self.run_command([self.command_name, "core", "--key-name", "key-name"])

        self.assertThat(result.exit_code, Equals(0))
        self.popen_mock.assert_called_with(
            ["snap", "sign", "-k", "key-name"], stderr=-1, stdin=-1, stdout=-1
        )

    def test_no_prior_developer_assertion(self):
        result = self.run_command(
            [self.command_name, "core-no-dev", "--key-name", "key-name"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.popen_mock.assert_called_with(
            ["snap", "sign", "-k", "key-name"], stderr=-1, stdin=-1, stdout=-1
        )

    def test_collaborate_no_revoke_uploads_request(self):
        result = self.run_command(
            [self.command_name] + ["no-revoked", "--key-name", "key-name"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains("This will revoke the following collaborators: 'this'"),
        )
        self.assertThat(
            result.output, Contains("Are you sure you want to continue? [y/N]:")
        )
        self.assertThat(
            result.output,
            Contains("The collaborators for this snap have not been altered."),
        )

    def test_collaborate_unchanged_collaborators(self):
        self.edit_developers_mock.return_value = [
            {
                "developer-id": "test-dev-id",
                "since": "2017-02-10 08:35:00",
                "until": "2018-02-10 08:35:00",
            }
        ]

        result = self.run_command(
            [self.command_name] + ["test-snap-with-dev", "--key-name", "key-name"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output, Contains("Aborting due to unchanged collaborators list.")
        )


class CollaborateErrorsTestCase(CollaborateBaseTestCase):

    scenarios = [
        ("edit-collaborators", {"command_name": "edit-collaborators"}),
        ("collaborators alias", {"command_name": "collaborators"}),
    ]

    def test_collaborate_snap_not_found(self):
        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            [self.command_name, "notfound", "--key-name", "key-name"],
        )

        self.assertThat(str(raised), Contains("Snap 'notfound' was not found"))

    def test_collaborate_bad_request(self):
        raised = self.assertRaises(
            storeapi.errors.StoreValidationError,
            self.run_command,
            [self.command_name, "badrequest", "--key-name", "key-name"],
        )

        self.assertThat(
            str(raised),
            Contains(
                "Received error 400: 'The given `snap-id` does not match "
                "the assertion.'"
            ),
        )

    def test_collaborate_yes_revoke_uploads_request(self):
        raised = self.assertRaises(
            storeapi.errors.StoreValidationError,
            self.run_command,
            [self.command_name, "revoked", "--key-name", "key-name"],
            input="y\n",
        )

        self.assertThat(
            str(raised),
            Contains(
                'Received error 409: "{}"'.format(
                    "The assertion's `developers` would revoke existing uploads."
                )
            ),
        )


class EditDevelopersTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        patcher = mock.patch("subprocess.check_call")
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_edit_developers_must_write_header_and_developers(self):
        developers_from_assertion = [
            {
                "developer-id": "test-dev-id1",
                "since": "2017-02-10T08:35:00.000000Z",
                "until": "2018-02-10T08:35:00.000000Z",
            },
            {
                "developer-id": "test-dev-id2",
                "since": "2016-02-10T08:35:00.000000Z",
                "until": "2019-02-10T08:35:00.000000Z",
            },
        ]

        existing_developers = (
            "developers:\n"
            "- developer-id: test-dev-id1\n"
            "  since: '2017-02-10 08:35:00'\n"
            "  until: '2018-02-10 08:35:00'\n"
            "- developer-id: test-dev-id2\n"
            "  since: '2016-02-10 08:35:00'\n"
            "  until: '2019-02-10 08:35:00'\n"
        )
        expected_written = assertions._COLLABORATION_HEADER + "\n" + existing_developers

        with mock.patch(
            "builtins.open", new_callable=mock.mock_open, read_data=expected_written
        ) as mock_open:
            developers_for_assertion = assertions._update_developers(
                developers_from_assertion
            )

        written = ""
        for call in mock_open().write.call_args_list:
            written += str(call.call_list()[0][0][0])
        self.assertThat(written, Equals(expected_written))
        self.assertThat(developers_for_assertion, Equals(developers_from_assertion))

    def test_edit_developers_must_return_new_developers(self):
        developers_from_assertion = [
            {
                "developer-id": "test-dev-id1",
                "since": "2017-02-10T08:35:00.000000Z",
                "until": "2018-02-10T08:35:00.000000Z",
            }
        ]

        new_developers = (
            "developers:\n"
            "- developer-id: test-dev-id1\n"
            "  since: '2017-02-10 08:35:00'\n"
            "  until: '2018-02-10 08:35:00'\n"
            "- developer-id: test-dev-id2\n"
            "  since: '2016-02-10 08:35:00'\n"
            "  until: '2019-02-10 08:35:00'\n"
        )

        with mock.patch(
            "builtins.open", new_callable=mock.mock_open, read_data=new_developers
        ):
            developers_for_assertion = assertions._update_developers(
                developers_from_assertion
            )

        expected_developers = developers_from_assertion + [
            {
                "developer-id": "test-dev-id2",
                "since": "2016-02-10T08:35:00.000000Z",
                "until": "2019-02-10T08:35:00.000000Z",
            }
        ]
        self.assertThat(developers_for_assertion, Equals(expected_developers))


class EditDevelopersOpenEditorTestCase(unit.TestCase):

    scenarios = (
        ("default", {"editor": None, "expected": "vi"}),
        ("non-default", {"editor": "test-editor", "expected": "test-editor"}),
    )

    def setUp(self):
        super().setUp()
        patcher = mock.patch("subprocess.check_call")
        self.check_call_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_edit_collaborators_must_open_editor(self):
        self.useFixture(fixtures.EnvironmentVariable("EDITOR", self.editor))
        assertions._edit_developers({})
        self.check_call_mock.assert_called_with([self.expected, mock.ANY])
