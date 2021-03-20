# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

from simplejson.scanner import JSONDecodeError
from testtools.matchers import Contains, Equals, Not

from snapcraft import storeapi

from . import FakeStoreCommandsBaseTestCase


class RegisterTestCase(FakeStoreCommandsBaseTestCase):
    def test_register_without_name_must_error(self):
        result = self.run_command(["register"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_register_without_login_must_ask(self):
        self.fake_store_register.mock.side_effect = [
            storeapi.http_clients.errors.InvalidCredentialsError("error"),
            None,
        ]

        result = self.run_command(
            ["register", "snap-name"], input="y\nuser@example.com\nsecret\n"
        )
        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )

    def test_register_name_successfully(self):
        result = self.run_command(["register", "test-snap"], input="y\n")

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Registering test-snap"))
        self.assertThat(
            result.output,
            Contains("Congrats! You are now the publisher of 'test-snap'."),
        )
        self.assertThat(
            result.output,
            Not(Contains("Congratulations! You're now the publisher for 'test-snap'.")),
        )
        self.fake_store_register.mock.assert_called_once_with(
            "test-snap", is_private=False, series="16", store_id=None
        )

    def test_register_name_to_specific_store_successfully(self):
        result = self.run_command(
            ["register", "test-snap", "--store", "my-brand"], input="y\n"
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Registering test-snap"))
        self.assertThat(
            result.output,
            Contains("Congrats! You are now the publisher of 'test-snap'."),
        )
        self.assertThat(
            result.output,
            Not(Contains("Congratulations! You're now the publisher for 'test-snap'.")),
        )
        self.fake_store_register.mock.assert_called_once_with(
            "test-snap", is_private=False, series="16", store_id="my-brand"
        )

    def test_register_private_name_successfully(self):
        result = self.run_command(["register", "test-snap", "--private"], input="y\n")

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains("Even though this is private snap, you should think carefully"),
        )
        self.assertThat(result.output, Contains("Registering test-snap"))
        self.assertThat(
            result.output,
            Contains("Congrats! You are now the publisher of 'test-snap'."),
        )
        self.assertThat(
            result.output,
            Not(Contains("Congratulations! You're now the publisher for 'test-snap'.")),
        )
        self.fake_store_register.mock.assert_called_once_with(
            "test-snap", is_private=True, series="16", store_id=None
        )

    def test_registration_failed(self):
        response = mock.Mock()
        response.json.side_effect = JSONDecodeError("mock-fail", "doc", 1)
        self.fake_store_register.mock.side_effect = storeapi.errors.StoreRegistrationError(
            "test-snap", response
        )

        raised = self.assertRaises(
            storeapi.errors.StoreRegistrationError,
            self.run_command,
            ["register", "test-snap"],
            input="y\n",
        )

        self.assertThat(str(raised), Equals("Registration failed."))

    def test_registration_cancelled(self):
        response = mock.Mock()
        response.json.side_effect = JSONDecodeError("mock-fail", "doc", 1)
        self.fake_store_register.mock.side_effect = storeapi.errors.StoreRegistrationError(
            "test-snap", response
        )

        result = self.run_command(["register", "test-snap"], input="n\n")

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output, Contains("Thank you! 'test-snap' will remain available")
        )
