# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2018 Canonical Ltd
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

import json
import logging
import os
import tempfile
from textwrap import dedent
from unittest import mock

import fixtures
import pymacaroons
from testtools.matchers import Contains, Equals

from snapcraft import config, storeapi, ProjectOptions
from snapcraft.storeapi import errors, constants
import tests
from tests import fixture_setup, unit


class StoreTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()


class LoginTestCase(StoreTestCase):
    def test_login_successful(self):
        self.client.login("dummy email", "test correct password")
        conf = config.Config()
        self.assertIsNotNone(conf.get("macaroon"))
        self.assertIsNotNone(conf.get("unbound_discharge"))

    def test_login_successful_with_one_time_password(self):
        self.client.login(
            "dummy email", "test correct password", "test correct one-time password"
        )
        conf = config.Config()
        self.assertIsNotNone(conf.get("macaroon"))
        self.assertIsNotNone(conf.get("unbound_discharge"))

    def test_login_successful_with_package_attenuation(self):
        self.client.login(
            "dummy email",
            "test correct password",
            packages=[{"name": "foo", "series": "16"}],
        )
        conf = config.Config()
        self.assertIsNotNone(conf.get("macaroon"))
        self.assertIsNotNone(conf.get("unbound_discharge"))

    def test_login_successful_with_channel_attenuation(self):
        self.client.login("dummy email", "test correct password", channels=["edge"])
        conf = config.Config()
        self.assertIsNotNone(conf.get("macaroon"))
        self.assertIsNotNone(conf.get("unbound_discharge"))

    def test_login_successful_fully_attenuated(self):
        self.client.login(
            "dummy email",
            "test correct password",
            packages=[{"name": "foo", "series": "16"}],
            channels=["edge"],
            save=False,
        )
        # Client configuration is filled, but it's not saved on disk.
        self.assertIsNotNone(self.client.conf.get("macaroon"))
        self.assertIsNotNone(self.client.conf.get("unbound_discharge"))
        self.assertTrue(config.Config().is_empty())

    def test_login_successful_with_expiration(self):
        self.client.login(
            "dummy email",
            "test correct password",
            packages=[{"name": "foo", "series": "16"}],
            channels=["edge"],
            expires="2017-12-22",
        )
        self.assertIsNotNone(self.client.conf.get("macaroon"))
        self.assertIsNotNone(self.client.conf.get("unbound_discharge"))

    def test_login_with_exported_login(self):
        conf = config.Config()
        conf.set("macaroon", "test-macaroon")
        conf.set("unbound_discharge", "test-unbound-discharge")
        with open("test-exported-login", "w+") as config_fd:
            conf.save(config_fd=config_fd)
            config_fd.seek(0)
            self.client.login("", "", config_fd=config_fd)

        # Client configuration is filled, but it's not saved on disk.
        self.assertThat(self.client.conf.get("macaroon"), Equals("test-macaroon"))
        self.assertThat(
            self.client.conf.get("unbound_discharge"), Equals("test-unbound-discharge")
        )

    def test_failed_login_with_wrong_password(self):
        self.assertRaises(
            errors.StoreAuthenticationError,
            self.client.login,
            "dummy email",
            "wrong password",
        )

        self.assertTrue(config.Config().is_empty())

    def test_failed_login_requires_one_time_password(self):
        self.assertRaises(
            errors.StoreTwoFactorAuthenticationRequired,
            self.client.login,
            "dummy email",
            "test requires 2fa",
        )

        self.assertTrue(config.Config().is_empty())

    def test_failed_login_with_wrong_one_time_password(self):
        self.assertRaises(
            errors.StoreAuthenticationError,
            self.client.login,
            "dummy email",
            "test correct password",
            "wrong one-time password",
        )

        self.assertTrue(config.Config().is_empty())

    def test_failed_login_with_invalid_json(self):
        self.assertRaises(
            errors.StoreAuthenticationError,
            self.client.login,
            "dummy email",
            "test 401 invalid json",
        )

        self.assertTrue(config.Config().is_empty())

    def test_failed_login_with_unregistered_snap(self):
        raised = self.assertRaises(
            errors.StoreAuthenticationError,
            self.client.login,
            "dummy email",
            "test correct password",
            packages=[{"name": "unregistered-snap-name", "series": "16"}],
        )

        self.assertThat(str(raised), Contains("not found"))

        self.assertTrue(config.Config().is_empty())


class DownloadTestCase(StoreTestCase):

    # sha512 of tests/data/test-snap.snap
    EXPECTED_SHA512 = (
        "69D57DCACF4F126592D4E6FF689AD8BB8A083C7B9FE44F6E738EF"
        "d22a956457f14146f7f067b47bd976cf0292f2993ad864ccb498b"
        "fda4128234e4c201f28fe9"
    )

    def test_download_unexisting_snap_raises_exception(self):
        self.client.login("dummy", "test correct password")
        e = self.assertRaises(
            errors.SnapNotFoundError,
            self.client.download,
            "unexisting-snap",
            "test-channel",
            "dummy",
            "test-arch",
        )
        self.assertThat(
            str(e),
            Equals(
                "Snap 'unexisting-snap' for 'test-arch' cannot be found in "
                "the 'test-channel' channel."
            ),
        )

    def test_download_snap(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.client.login("dummy", "test correct password")
        download_path = os.path.join(self.path, "test-snap.snap")
        self.client.download("test-snap", "test-channel", download_path)
        self.assertIn(
            "Successfully downloaded test-snap at {}".format(download_path),
            self.fake_logger.output,
        )

    def test_download_from_branded_store_requires_login(self):
        err = self.assertRaises(
            errors.SnapNotFoundError,
            self.client.download,
            "test-snap-branded-store",
            "test-channel",
            "dummy",
        )

        arch = ProjectOptions().deb_arch
        self.assertThat(
            str(err),
            Equals(
                "Snap 'test-snap-branded-store' for '{}' cannot be found in "
                "the 'test-channel' channel.".format(arch)
            ),
        )

    def test_download_from_branded_store_requires_store(self):
        self.client.login("dummy", "test correct password")
        err = self.assertRaises(
            errors.SnapNotFoundError,
            self.client.download,
            "test-snap-branded-store",
            "test-channel",
            "dummy",
        )

        arch = ProjectOptions().deb_arch
        self.assertThat(
            str(err),
            Equals(
                "Snap 'test-snap-branded-store' for '{}' cannot be found in "
                "the 'test-channel' channel.".format(arch)
            ),
        )

    def test_download_from_branded_store(self):
        # Downloading from a branded-store requires login (authorization)
        # and setting 'SNAPCRAFT_UBUNTU_STORE' environment variable to the
        # correct store 'slug' (the branded store identifier).
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_UBUNTU_STORE", "Test-Branded")
        )
        self.client.login("dummy", "test correct password")

        download_path = os.path.join(self.path, "brand.snap")
        self.client.download("test-snap-branded-store", "test-channel", download_path)

        self.assertIn(
            "Successfully downloaded test-snap-branded-store at {}".format(
                download_path
            ),
            self.fake_logger.output,
        )

    def test_download_already_downloaded_snap(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.client.login("dummy", "test correct password")
        download_path = os.path.join(self.path, "test-snap.snap")
        # download first time.
        self.client.download("test-snap", "test-channel", download_path)
        # download again.
        self.client.download("test-snap", "test-channel", download_path)
        self.assertIn(
            "Already downloaded test-snap at {}".format(download_path),
            self.fake_logger.output,
        )

    def test_download_on_sha_mismatch(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.client.login("dummy", "test correct password")
        download_path = os.path.join(self.path, "test-snap.snap")
        # Write a wrong file in the download path.
        open(download_path, "w").close()
        self.client.download("test-snap", "test-channel", download_path)
        self.assertIn(
            "Successfully downloaded test-snap at {}".format(download_path),
            self.fake_logger.output,
        )

    def test_download_with_hash_mismatch_raises_exception(self):
        self.client.login("dummy", "test correct password")
        download_path = os.path.join(self.path, "test-snap.snap")
        self.assertRaises(
            errors.SHAMismatchError,
            self.client.download,
            "test-snap-with-wrong-sha",
            "test-channel",
            download_path,
        )


class PushSnapBuildTestCase(StoreTestCase):
    def test_push_snap_build_without_login_raises_exception(self):
        self.assertRaises(
            errors.InvalidCredentialsError,
            self.client.push_snap_build,
            "snap-id",
            "dummy",
        )

    def test_push_snap_build_refreshes_macaroon(self):
        self.client.login("dummy", "test correct password")
        self.fake_store.needs_refresh = True
        self.client.push_snap_build("snap-id", "dummy")
        self.assertFalse(self.fake_store.needs_refresh)

    def test_push_snap_build_not_implemented(self):
        # If the "enable_snap_build" feature switch is off in the store, we
        # will get a descriptive error message.
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreServerError,
            self.client.push_snap_build,
            "snap-id",
            "test-not-implemented",
        )
        self.assertThat(raised.error_code, Equals(501))

    def test_push_snap_build_invalid_data(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreSnapBuildError,
            self.client.push_snap_build,
            "snap-id",
            "test-invalid-data",
        )
        self.assertThat(
            str(raised),
            Equals("Could not assert build: The snap-build assertion is not " "valid."),
        )

    def test_push_snap_build_unexpected_data(self):
        # The endpoint in SCA would never return plain/text, however anything
        # might happen in the internet, so we are a little defensive.
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreServerError,
            self.client.push_snap_build,
            "snap-id",
            "test-unexpected-data",
        )
        self.assertThat(raised.error_code, Equals(500))

    def test_push_snap_build_successfully(self):
        self.client.login("dummy", "test correct password")
        # No exception will be raised if this is successful.
        self.client.push_snap_build("snap-id", "dummy")


class GetAccountInformationTestCase(StoreTestCase):
    def test_get_account_information_without_login_raises_exception(self):
        self.assertRaises(
            errors.InvalidCredentialsError, self.client.get_account_information
        )

    def test_get_account_information_successfully(self):
        self.client.login("dummy", "test correct password")
        self.assertThat(
            self.client.get_account_information(),
            Equals(
                {
                    "account_id": "abcd",
                    "account_keys": [],
                    "snaps": {
                        "16": {
                            "basic": {
                                "snap-id": "snap-id",
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                            "core": {
                                "snap-id": "good",
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                            "core-no-dev": {
                                "snap-id": "no-dev",
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                            "badrequest": {
                                "snap-id": "badrequest",
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                            "no-revoked": {
                                "snap-id": "no-revoked",
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                            "revoked": {
                                "snap-id": "revoked",
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                            "test-snap-with-dev": {
                                "price": None,
                                "private": False,
                                "since": "2016-12-12T01:01:01Z",
                                "snap-id": "test-snap-id-with-dev",
                                "status": "Approved",
                            },
                            "test-snap-with-no-validations": {
                                "price": None,
                                "private": False,
                                "since": "2016-12-12T01:01:01Z",
                                "snap-id": "test-snap-id-with-no-validations",
                                "status": "Approved",
                            },
                            "no-id": {
                                "snap-id": None,
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                        }
                    },
                }
            ),
        )

    def test_get_account_information_refreshes_macaroon(self):
        self.client.login("dummy", "test correct password")
        self.fake_store.needs_refresh = True
        self.assertThat(
            self.client.get_account_information(),
            Equals(
                {
                    "account_id": "abcd",
                    "account_keys": [],
                    "snaps": {
                        "16": {
                            "basic": {
                                "snap-id": "snap-id",
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                            "core": {
                                "snap-id": "good",
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                            "core-no-dev": {
                                "snap-id": "no-dev",
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                            "badrequest": {
                                "snap-id": "badrequest",
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                            "no-revoked": {
                                "snap-id": "no-revoked",
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                            "revoked": {
                                "snap-id": "revoked",
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                            "test-snap-with-dev": {
                                "price": None,
                                "private": False,
                                "since": "2016-12-12T01:01:01Z",
                                "snap-id": "test-snap-id-with-dev",
                                "status": "Approved",
                            },
                            "test-snap-with-no-validations": {
                                "price": None,
                                "private": False,
                                "since": "2016-12-12T01:01:01Z",
                                "snap-id": "test-snap-id-with-no-validations",
                                "status": "Approved",
                            },
                            "no-id": {
                                "snap-id": None,
                                "status": "Approved",
                                "private": False,
                                "price": None,
                                "since": "2016-12-12T01:01:01Z",
                            },
                        }
                    },
                }
            ),
        )
        self.assertFalse(self.fake_store.needs_refresh)


class RegisterKeyTestCase(StoreTestCase):
    def test_register_key_without_login_raises_exception(self):
        self.assertRaises(
            errors.InvalidCredentialsError, self.client.register_key, "dummy"
        )

    def test_register_key_successfully(self):
        self.client.login("dummy", "test correct password")
        # No exception will be raised if this is successful.
        self.client.register_key(
            dedent(
                """\
            name: default
            public-key-sha3-384: abcd
            """
            )
        )

    def test_register_key_refreshes_macaroon(self):
        self.client.login("dummy", "test correct password")
        self.fake_store.needs_refresh = True
        self.client.register_key(
            dedent(
                """\
            name: default
            public-key-sha3-384: abcd
            """
            )
        )
        self.assertFalse(self.fake_store.needs_refresh)

    def test_not_implemented(self):
        # If the enable_account_key feature switch is off in the store, we
        # will get a 501 Not Implemented response.
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreServerError, self.client.register_key, "test-not-implemented"
        )
        self.assertThat(raised.error_code, Equals(501))

    def test_invalid_data(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreKeyRegistrationError,
            self.client.register_key,
            "test-invalid-data",
        )
        self.assertThat(
            str(raised),
            Equals(
                "Key registration failed: "
                "The account-key-request assertion is not valid."
            ),
        )


class RegisterTestCase(StoreTestCase):
    def test_register_without_login_raises_exception(self):
        self.assertRaises(errors.InvalidCredentialsError, self.client.register, "dummy")

    def test_register_name_successfully(self):
        self.client.login("dummy", "test correct password")
        # No exception will be raised if this is successful
        self.client.register("test-good-snap-name")

    def test_register_private_name_successfully(self):
        self.client.login("dummy", "test correct password")
        # No exception will be raised if this is successful
        self.client.register("test-good-snap-name", is_private=True)

    def test_register_refreshes_macaroon(self):
        self.client.login("dummy", "test correct password")
        self.fake_store.needs_refresh = True
        self.client.register("test-good-snap-name")
        self.assertFalse(self.fake_store.needs_refresh)

    def test_already_registered(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreRegistrationError,
            self.client.register,
            "test-snap-name-already-registered",
        )
        self.assertThat(
            str(raised),
            Equals(
                "The name 'test-snap-name-already-registered' is already "
                "taken.\n\n"
                "We can if needed rename snaps to ensure they match the "
                "expectations of most users. If you are the publisher most "
                "users expect for 'test-snap-name-already-registered' then "
                "claim the name at 'https://myapps.com/register-name/'"
            ),
        )

    def test_register_a_reserved_name(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreRegistrationError,
            self.client.register,
            "test-reserved-snap-name",
        )
        self.assertThat(
            str(raised),
            Equals(
                "The name 'test-reserved-snap-name' is reserved."
                "\n\n"
                "If you are the publisher most users expect for "
                "'test-reserved-snap-name' then please claim the "
                "name at 'https://myapps.com/register-name/'\n\n"
                "Otherwise, please register another name."
            ),
        )

    def test_register_already_owned_name(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreRegistrationError,
            self.client.register,
            "test-already-owned-snap-name",
        )
        self.assertThat(
            str(raised),
            Equals("You already own the name 'test-already-owned-snap-name'."),
        )

    def test_registering_too_fast_in_a_row(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreRegistrationError, self.client.register, "test-snapcraft-fast"
        )
        self.assertThat(
            str(raised),
            Equals(
                "You must wait 177 seconds before trying to register your " "next snap."
            ),
        )

    def test_registering_name_too_long(self):
        self.client.login("dummy", "test correct password")
        name = "name-too-l{}ng".format("0" * 40)
        raised = self.assertRaises(
            errors.StoreRegistrationError, self.client.register, name
        )
        expected = (
            "The name '{}' is not valid: it should be no longer than 40"
            " characters.".format(name)
        )
        self.assertThat(str(raised), Equals(expected))

    def test_registering_name_invalid(self):
        self.client.login("dummy", "test correct password")
        name = "test_invalid"
        raised = self.assertRaises(
            errors.StoreRegistrationError, self.client.register, name
        )
        expected = (
            "The name '{}' is not valid: it should only have"
            " ASCII lowercase letters, numbers, and hyphens,"
            " and must have at least one letter.".format(name)
        )
        self.assertThat(str(raised), Equals(expected))

    def test_unhandled_registration_error_path(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreRegistrationError,
            self.client.register,
            "snap-name-no-clear-error",
        )
        self.assertThat(str(raised), Equals("Registration failed."))


class ValidationsTestCase(StoreTestCase):
    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)

    def test_get_success(self):
        self.client.login("dummy", "test correct password")
        expected = [
            {
                "approved-snap-id": "snap-id-1",
                "approved-snap-revision": "3",
                "approved-snap-name": "snap-1",
                "authority-id": "dev-1",
                "series": "16",
                "sign-key-sha3-384": "1234567890",
                "snap-id": "snap-id-gating",
                "timestamp": "2016-09-19T21:07:27.756001Z",
                "type": "validation",
                "revoked": "false",
                "required": True,
            },
            {
                "approved-snap-id": "snap-id-2",
                "approved-snap-revision": "5",
                "approved-snap-name": "snap-2",
                "authority-id": "dev-1",
                "series": "16",
                "sign-key-sha3-384": "1234567890",
                "snap-id": "snap-id-gating",
                "timestamp": "2016-09-19T21:07:27.756001Z",
                "type": "validation",
                "revoked": "false",
                "required": False,
            },
            {
                "approved-snap-id": "snap-id-3",
                "approved-snap-revision": "-",
                "approved-snap-name": "snap-3",
                "authority-id": "dev-1",
                "series": "16",
                "sign-key-sha3-384": "1234567890",
                "snap-id": "snap-id-gating",
                "timestamp": "2016-09-19T21:07:27.756001Z",
                "type": "validation",
                "revoked": "false",
                "required": True,
            },
        ]
        result = self.client.get_assertion("good", "validations")
        self.assertThat(result, Equals(expected))

    def test_get_bad_response(self):
        self.client.login("dummy", "test correct password")

        err = self.assertRaises(
            errors.StoreValidationError, self.client.get_assertion, "bad", "validations"
        )

        expected = "Received error 200: 'Invalid response from the server'"
        self.assertThat(str(err), Equals(expected))
        self.assertIn("Invalid response from the server", self.fake_logger.output)

    def test_get_error_response(self):
        self.client.login("dummy", "test correct password")

        err = self.assertRaises(
            errors.StoreNetworkError, self.client.get_assertion, "err", "validations"
        )

        expected = "maximum retries exceeded"
        self.assertThat(str(err), Contains(expected))

    def test_push_success(self):
        self.client.login("dummy", "test correct password")
        assertion = json.dumps({"foo": "bar"}).encode("utf-8")

        result = self.client.push_assertion("good", assertion, "validations")

        expected = {"assertion": '{"foo": "bar"}'}
        self.assertThat(result, Equals(expected))

    def test_push_bad_response(self):
        self.client.login("dummy", "test correct password")
        assertion = json.dumps({"foo": "bar"}).encode("utf-8")

        err = self.assertRaises(
            errors.StoreValidationError,
            self.client.push_assertion,
            "bad",
            assertion,
            "validations",
        )

        expected = "Received error 200: 'Invalid response from the server'"
        self.assertThat(str(err), Equals(expected))
        self.assertIn("Invalid response from the server", self.fake_logger.output)

    def test_push_error_response(self):
        self.client.login("dummy", "test correct password")
        assertion = json.dumps({"foo": "bar"}).encode("utf-8")

        err = self.assertRaises(
            errors.StoreServerError,
            self.client.push_assertion,
            "err",
            assertion,
            "validations",
        )
        self.assertThat(err.error_code, Equals(501))


class UploadTestCase(StoreTestCase):
    def setUp(self):
        super().setUp()
        self.snap_path = os.path.join(
            os.path.dirname(tests.__file__), "data", "test-snap.snap"
        )
        # These should eventually converge to the same module
        pbars = (
            "snapcraft.storeapi._upload.ProgressBar",
            "snapcraft.storeapi._status_tracker.ProgressBar",
        )
        for pbar in pbars:
            patcher = mock.patch(pbar, new=unit.SilentProgressBar)
            patcher.start()
            self.addCleanup(patcher.stop)

    def test_upload_without_login_raises_exception(self):
        self.assertRaises(
            errors.InvalidCredentialsError,
            self.client.upload,
            "test-snap",
            self.snap_path,
        )

    def test_upload_snap(self):
        self.client.login("dummy", "test correct password")
        self.client.register("test-snap")
        tracker = self.client.upload("test-snap", self.snap_path)
        self.assertTrue(isinstance(tracker, storeapi._status_tracker.StatusTracker))
        result = tracker.track()
        expected_result = {
            "code": "ready_to_release",
            "revision": "1",
            "url": "/dev/click-apps/5349/rev/1",
            "can_release": True,
            "processed": True,
        }
        self.assertThat(result, Equals(expected_result))

        # This should not raise
        tracker.raise_for_code()

    def test_upload_refreshes_macaroon(self):
        self.client.login("dummy", "test correct password")
        self.client.register("test-snap")
        self.fake_store.needs_refresh = True
        tracker = self.client.upload("test-snap", self.snap_path)
        result = tracker.track()
        expected_result = {
            "code": "ready_to_release",
            "revision": "1",
            "url": "/dev/click-apps/5349/rev/1",
            "can_release": True,
            "processed": True,
        }
        self.assertThat(result, Equals(expected_result))

        # This should not raise
        tracker.raise_for_code()

        self.assertFalse(self.fake_store.needs_refresh)

    def test_upload_snap_fails_due_to_upload_fail(self):
        # Tells the fake updown server to return a 5xx response
        self.useFixture(fixtures.EnvironmentVariable("UPDOWN_BROKEN", "1"))

        self.client.login("dummy", "test correct password")

        raised = self.assertRaises(
            errors.StoreServerError, self.client.upload, "test-snap", self.snap_path
        )
        self.assertThat(raised.error_code, Equals(500))

    def test_upload_snap_requires_review(self):
        self.client.login("dummy", "test correct password")
        self.client.register("test-review-snap")
        tracker = self.client.upload("test-review-snap", self.snap_path)
        self.assertTrue(isinstance(tracker, storeapi._status_tracker.StatusTracker))
        result = tracker.track()
        expected_result = {
            "code": "need_manual_review",
            "revision": "1",
            "url": "/dev/click-apps/5349/rev/1",
            "can_release": False,
            "processed": True,
        }
        self.assertThat(result, Equals(expected_result))

        self.assertRaises(errors.StoreReviewError, tracker.raise_for_code)

    def test_upload_duplicate_snap(self):
        self.client.login("dummy", "test correct password")
        self.client.register("test-duplicate-snap")
        tracker = self.client.upload("test-duplicate-snap", self.snap_path)
        self.assertTrue(isinstance(tracker, storeapi._status_tracker.StatusTracker))
        result = tracker.track()
        expected_result = {
            "code": "processing_error",
            "revision": "1",
            "url": "/dev/click-apps/5349/rev/1",
            "can_release": False,
            "processed": True,
            "errors": [{"message": "Duplicate snap already uploaded"}],
        }
        self.assertThat(result, Equals(expected_result))

        raised = self.assertRaises(errors.StoreReviewError, tracker.raise_for_code)

        self.assertThat(
            str(raised),
            Equals(
                "The store was unable to accept this snap.\n"
                "  - Duplicate snap already uploaded"
            ),
        )

    def test_braces_in_error_messages_are_literals(self):
        self.client.login("dummy", "test correct password")
        self.client.register("test-scan-error-with-braces")
        tracker = self.client.upload("test-scan-error-with-braces", self.snap_path)
        self.assertTrue(isinstance(tracker, storeapi._status_tracker.StatusTracker))
        result = tracker.track()
        expected_result = {
            "code": "processing_error",
            "revision": "1",
            "url": "/dev/click-apps/5349/rev/1",
            "can_release": False,
            "processed": True,
            "errors": [{"message": "Error message with {braces}"}],
        }
        self.assertThat(result, Equals(expected_result))

        raised = self.assertRaises(errors.StoreReviewError, tracker.raise_for_code)

        self.assertThat(
            str(raised),
            Equals(
                "The store was unable to accept this snap.\n"
                "  - Error message with {braces}"
            ),
        )

    def test_push_unregistered_snap(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StorePushError,
            self.client.upload,
            "test-snap-unregistered",
            self.snap_path,
        )
        self.assertThat(
            str(raised),
            Equals(
                "You are not the publisher or allowed to push revisions for "
                "this snap. To become the publisher, run `snapcraft register "
                "test-snap-unregistered` and try to push again."
            ),
        )

    def test_upload_with_invalid_credentials_raises_exception(self):
        conf = config.Config()
        conf.set("macaroon", 'inval"id')
        conf.save()
        self.assertRaises(
            errors.InvalidCredentialsError,
            self.client.upload,
            "test-snap",
            self.snap_path,
        )


class ReleaseTestCase(StoreTestCase):
    def test_release_without_login_raises_exception(self):
        self.assertRaises(
            errors.InvalidCredentialsError,
            self.client.release,
            "test-snap",
            "19",
            ["beta"],
        )

    def test_release_snap(self):
        self.client.login("dummy", "test correct password")
        channel_map = self.client.release("test-snap", "19", ["beta"])
        expected_channel_map = {
            "opened_channels": ["beta"],
            "channel_map": [
                {"channel": "stable", "info": "none"},
                {"channel": "candidate", "info": "none"},
                {"revision": 19, "channel": "beta", "version": "0", "info": "specific"},
                {"channel": "edge", "info": "tracking"},
            ],
        }
        self.assertThat(channel_map, Equals(expected_channel_map))

    def test_release_refreshes_macaroon(self):
        self.client.login("dummy", "test correct password")
        self.fake_store.needs_refresh = True
        channel_map = self.client.release("test-snap", "19", ["beta"])
        expected_channel_map = {
            "opened_channels": ["beta"],
            "channel_map": [
                {"channel": "stable", "info": "none"},
                {"channel": "candidate", "info": "none"},
                {"revision": 19, "channel": "beta", "version": "0", "info": "specific"},
                {"channel": "edge", "info": "tracking"},
            ],
        }
        self.assertThat(channel_map, Equals(expected_channel_map))
        self.assertFalse(self.fake_store.needs_refresh)

    def test_release_snap_to_invalid_channel(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreReleaseError, self.client.release, "test-snap", "19", ["alpha"]
        )

        self.assertThat(str(raised), Equals("Not a valid channel: alpha"))

    def test_release_snap_to_bad_channel(self):
        self.client.login("dummy", "test correct password")
        self.assertRaises(
            errors.StoreServerError,
            self.client.release,
            "test-snap",
            "19",
            ["bad-channel"],
        )

    def test_release_unregistered_snap(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreReleaseError,
            self.client.release,
            "test-snap-unregistered",
            "19",
            ["alpha"],
        )

        self.assertThat(
            str(raised),
            Equals(
                "Sorry, try `snapcraft register test-snap-unregistered` "
                "before trying to release or choose an existing "
                "revision."
            ),
        )

    def test_release_with_invalid_credentials_raises_exception(self):
        conf = config.Config()
        conf.set("macaroon", 'inval"id')
        conf.save()
        self.assertRaises(
            errors.InvalidCredentialsError,
            self.client.release,
            "test-snap",
            "10",
            ["beta"],
        )

    def test_release_with_invalid_revision(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreReleaseError,
            self.client.release,
            "test-snap-invalid-data",
            "notanumber",
            ["beta"],
        )

        self.assertThat(
            str(raised),
            Equals("invalid-field: The 'revision' field must be an integer"),
        )

    def test_release_to_curly_braced_channel(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreReleaseError,
            self.client.release,
            "test-snap",
            "19",
            ["edge/{curly}"],
        )

        self.assertThat(
            str(raised),
            Equals(
                "invalid-field: Invalid branch name: {curly}. Enter a value consisting of "
                "letters, numbers or hyphens. Hyphens cannot occur at the start or end of the "
                "chosen value."
            ),
        )


class CloseChannelsTestCase(StoreTestCase):
    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)

    def test_close_requires_login(self):
        self.assertRaises(
            errors.InvalidCredentialsError,
            self.client.close_channels,
            "snap-id",
            ["dummy"],
        )

    def test_close_refreshes_macaroon(self):
        self.client.login("dummy", "test correct password")
        self.fake_store.needs_refresh = True
        self.client.close_channels("snap-id", ["dummy"])
        self.assertFalse(self.fake_store.needs_refresh)

    def test_close_invalid_data(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreChannelClosingError,
            self.client.close_channels,
            "snap-id",
            ["invalid"],
        )
        self.assertThat(
            str(raised),
            Equals(
                "Could not close channel: The 'channels' field content " "is not valid."
            ),
        )

    def test_close_unexpected_data(self):
        # The endpoint in SCA would never return plain/text, however anything
        # might happen in the internet, so we are a little defensive.
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreServerError,
            self.client.close_channels,
            "snap-id",
            ["unexpected"],
        )
        self.assertThat(raised.error_code, Equals(500))

    def test_close_broken_store_plain(self):
        # If the contract is broken by the Store, users will be have additional
        # debug information available.
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreChannelClosingError,
            self.client.close_channels,
            "snap-id",
            ["broken-plain"],
        )
        self.assertThat(str(raised), Equals("Could not close channel: 200 OK"))

        expected_lines = [
            "Invalid response from the server on channel closing:",
            "200 OK",
            "b'plain data'",
        ]

        actual_lines = []
        for line in self.fake_logger.output.splitlines():
            line = line.strip()
            if line in expected_lines:
                actual_lines.append(line)

        self.assertThat(actual_lines, Equals(expected_lines))

    def test_close_broken_store_json(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreChannelClosingError,
            self.client.close_channels,
            "snap-id",
            ["broken-json"],
        )
        self.assertThat(str(raised), Equals("Could not close channel: 200 OK"))

        expected_lines = [
            "Invalid response from the server on channel closing:",
            "200 OK",
            'b\'{"closed_channels": ["broken-json"]}\'',
        ]

        actual_lines = []
        for line in self.fake_logger.output.splitlines():
            line = line.strip()
            if line in expected_lines:
                actual_lines.append(line)

        self.assertThat(actual_lines, Equals(expected_lines))

    def test_close_successfully(self):
        # Successfully closing a channels returns 'closed_channels'
        # and 'channel_map_tree' from the Store.
        self.client.login("dummy", "test correct password")
        closed_channels, channel_map_tree = self.client.close_channels(
            "snap-id", ["beta"]
        )
        self.assertThat(closed_channels, Equals(["beta"]))
        self.assertThat(
            channel_map_tree,
            Equals(
                {
                    "latest": {
                        "16": {
                            "amd64": [
                                {"channel": "stable", "info": "none"},
                                {"channel": "candidate", "info": "none"},
                                {
                                    "channel": "beta",
                                    "info": "specific",
                                    "revision": 42,
                                    "version": "1.1",
                                },
                                {"channel": "edge", "info": "tracking"},
                            ]
                        }
                    }
                }
            ),
        )


class MacaroonsTestCase(unit.TestCase):
    def test_invalid_macaroon_root_raises_exception(self):
        conf = config.Config()
        conf.set("macaroon", 'inval"id')
        conf.save()
        self.assertRaises(errors.InvalidCredentialsError, storeapi._macaroon_auth, conf)

    def test_invalid_discharge_raises_exception(self):
        conf = config.Config()
        conf.set("macaroon", pymacaroons.Macaroon().serialize())
        conf.set("unbound_discharge", "inval*id")
        conf.save()
        self.assertRaises(errors.InvalidCredentialsError, storeapi._macaroon_auth, conf)


class GetSnapRevisionsTestCase(StoreTestCase):
    def setUp(self):
        super().setUp()
        self.expected = [
            {
                "series": ["16"],
                "channels": [],
                "version": "2.0.1",
                "timestamp": "2016-09-27T19:23:40Z",
                "current_channels": ["beta", "edge"],
                "arch": "i386",
                "revision": 2,
            },
            {
                "series": ["16"],
                "channels": ["stable", "edge"],
                "version": "2.0.2",
                "timestamp": "2016-09-27T18:38:43Z",
                "current_channels": ["stable", "candidate", "beta"],
                "arch": "amd64",
                "revision": 1,
            },
        ]

    def test_get_snap_revisions_without_login_raises_exception(self):
        self.assertRaises(
            errors.InvalidCredentialsError, self.client.get_snap_revisions, "basic"
        )

    def test_get_snap_revisions_successfully(self):
        self.client.login("dummy", "test correct password")
        self.assertThat(self.client.get_snap_revisions("basic"), Equals(self.expected))

    def test_get_snap_revisions_filter_by_series(self):
        self.client.login("dummy", "test correct password")
        self.assertThat(
            self.client.get_snap_revisions("basic", series="16"), Equals(self.expected)
        )

    def test_get_snap_revisions_filter_by_arch(self):
        self.client.login("dummy", "test correct password")
        self.assertThat(
            self.client.get_snap_revisions("basic", arch="amd64"),
            Equals([rev for rev in self.expected if rev["arch"] == "amd64"]),
        )

    def test_get_snap_revisions_filter_by_series_and_filter(self):
        self.client.login("dummy", "test correct password")
        self.assertThat(
            self.client.get_snap_revisions("basic", series="16", arch="amd64"),
            Equals(
                [
                    rev
                    for rev in self.expected
                    if "16" in rev["series"] and rev["arch"] == "amd64"
                ]
            ),
        )

    def test_get_snap_revisions_filter_by_unknown_series(self):
        self.client.login("dummy", "test correct password")
        e = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.client.get_snap_revisions,
            "basic",
            series="12",
        )
        self.assertThat(str(e), Equals("Snap 'basic' was not found in '12' series."))

    def test_get_snap_revisions_filter_by_unknown_arch(self):
        self.client.login("dummy", "test correct password")
        e = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.client.get_snap_revisions,
            "basic",
            arch="somearch",
        )
        self.assertThat(
            str(e), Equals("Snap 'basic' for 'somearch' was not found in '16' series.")
        )

    def test_get_snap_revisions_refreshes_macaroon(self):
        self.client.login("dummy", "test correct password")
        self.fake_store.needs_refresh = True
        self.assertThat(self.client.get_snap_revisions("basic"), Equals(self.expected))
        self.assertFalse(self.fake_store.needs_refresh)

    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    @mock.patch.object(storeapi._sca_client.SCAClient, "get")
    def test_get_snap_revisions_server_error(self, mock_sca_get, mock_account_info):
        mock_account_info.return_value = {
            "snaps": {"16": {"basic": {"snap-id": "my_snap_id"}}}
        }

        mock_sca_get.return_value = mock.Mock(
            ok=False, status_code=500, reason="Server error", json=lambda: {}
        )

        self.client.login("dummy", "test correct password")
        e = self.assertRaises(
            storeapi.errors.StoreSnapRevisionsError,
            self.client.get_snap_revisions,
            "basic",
        )
        self.assertThat(
            str(e),
            Equals(
                "Error fetching revisions of snap id 'my_snap_id' for "
                "'any arch' in '16' series: 500 Server error."
            ),
        )


class GetSnapStatusTestCase(StoreTestCase):
    def setUp(self):
        super().setUp()
        self.expected = {
            "channel_map_tree": {
                "latest": {
                    "16": {
                        "i386": [
                            {"info": "none", "channel": "stable"},
                            {"info": "none", "channel": "beta"},
                            {
                                "info": "specific",
                                "version": "1.0-i386",
                                "channel": "edge",
                                "revision": 3,
                            },
                        ],
                        "amd64": [
                            {
                                "info": "specific",
                                "version": "1.0-amd64",
                                "channel": "stable",
                                "revision": 2,
                            },
                            {
                                "info": "specific",
                                "version": "1.1-amd64",
                                "channel": "beta",
                                "revision": 4,
                            },
                            {"info": "tracking", "channel": "edge"},
                        ],
                    }
                }
            }
        }

    def test_get_snap_status_without_login_raises_exception(self):
        self.assertRaises(
            errors.InvalidCredentialsError, self.client.get_snap_status, "basic"
        )

    def test_get_snap_status_successfully(self):
        self.client.login("dummy", "test correct password")
        self.assertThat(self.client.get_snap_status("basic"), Equals(self.expected))

    def test_get_snap_status_filter_by_series(self):
        self.client.login("dummy", "test correct password")
        self.assertThat(
            self.client.get_snap_status("basic", series="16"), Equals(self.expected)
        )

    def test_get_snap_status_filter_by_arch(self):
        self.client.login("dummy", "test correct password")
        exp_arch = self.expected["channel_map_tree"]["latest"]["16"]["amd64"]
        self.assertThat(
            self.client.get_snap_status("basic", arch="amd64"),
            Equals({"channel_map_tree": {"latest": {"16": {"amd64": exp_arch}}}}),
        )

    def test_get_snap_status_filter_by_series_and_filter(self):
        self.client.login("dummy", "test correct password")
        exp_arch = self.expected["channel_map_tree"]["latest"]["16"]["amd64"]
        self.assertThat(
            self.client.get_snap_status("basic", series="16", arch="amd64"),
            Equals({"channel_map_tree": {"latest": {"16": {"amd64": exp_arch}}}}),
        )

    def test_get_snap_status_filter_by_unknown_series(self):
        self.client.login("dummy", "test correct password")
        e = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.client.get_snap_status,
            "basic",
            series="12",
        )
        self.assertThat(str(e), Equals("Snap 'basic' was not found in '12' series."))

    def test_get_snap_status_filter_by_unknown_arch(self):
        self.client.login("dummy", "test correct password")
        e = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.client.get_snap_status,
            "basic",
            arch="somearch",
        )
        self.assertThat(
            str(e), Equals("Snap 'basic' for 'somearch' was not found in '16' series.")
        )

    def test_get_snap_status_no_id(self):
        self.client.login("dummy", "test correct password")
        e = self.assertRaises(
            storeapi.errors.NoSnapIdError, self.client.get_snap_status, "no-id"
        )
        self.assertThat(e.snap_name, Equals("no-id"))

    def test_get_snap_status_refreshes_macaroon(self):
        self.client.login("dummy", "test correct password")
        self.fake_store.needs_refresh = True
        self.assertThat(self.client.get_snap_status("basic"), Equals(self.expected))
        self.assertFalse(self.fake_store.needs_refresh)

    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    @mock.patch.object(storeapi._sca_client.SCAClient, "get")
    def test_get_snap_status_server_error(self, mock_sca_get, mock_account_info):
        mock_account_info.return_value = {
            "snaps": {"16": {"basic": {"snap-id": "my_snap_id"}}}
        }

        mock_sca_get.return_value = mock.Mock(
            ok=False, status_code=500, reason="Server error", json=lambda: {}
        )

        self.client.login("dummy", "test correct password")
        e = self.assertRaises(
            storeapi.errors.StoreSnapStatusError, self.client.get_snap_status, "basic"
        )
        self.assertThat(
            str(e),
            Equals(
                "Error fetching status of snap id 'my_snap_id' for 'any arch' "
                "in '16' series: 500 Server error."
            ),
        )


class SignDeveloperAgreementTestCase(StoreTestCase):
    def test_sign_dev_agreement_success(self):
        self.client.login("dummy", "test correct password")
        response = {
            "content": {
                "latest_tos_accepted": True,
                "tos_url": "http://fake-url.com",
                "latest_tos_date": "2000-01-01",
                "accepted_tos_date": "2010-10-10",
            }
        }
        self.assertThat(
            response,
            Equals(self.client.sign_developer_agreement(latest_tos_accepted=True)),
        )

    def test_sign_dev_agreement_exception(self):
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.DeveloperAgreementSignError,
            self.client.sign_developer_agreement,
            False,
        )
        self.assertIn(
            "There was an error while signing developer agreement.\n"
            "Reason: 'Bad Request'\n",
            str(raised),
        )

    def test_sign_dev_agreement_exception_store_down(self):
        self.useFixture(fixtures.EnvironmentVariable("STORE_DOWN", "1"))
        self.client.login("dummy", "test correct password")
        raised = self.assertRaises(
            errors.StoreServerError,
            self.client.sign_developer_agreement,
            latest_tos_accepted=True,
        )
        self.assertThat(raised.error_code, Equals(500))


class PushMetadataTestCase(StoreTestCase):
    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)

    def _setup_snap(self):
        """Login, register and push a snap.

        These are all the previous steps needed to push metadata.
        """
        self.client.login("dummy", "test correct password")
        self.client.register("basic")
        path = os.path.join(os.path.dirname(tests.__file__), "data", "test-snap.snap")
        tracker = self.client.upload("basic", path)
        tracker.track()

    def test_requires_login(self):
        self.assertRaises(
            errors.InvalidCredentialsError,
            self.client.push_metadata,
            "basic",
            {},
            False,
        )

    def test_refreshes_macaroon(self):
        self._setup_snap()
        self.fake_store.needs_refresh = True
        metadata = {"field_ok": "foo"}
        self.client.push_metadata("basic", metadata, False)
        self.assertFalse(self.fake_store.needs_refresh)

    def test_invalid_data(self):
        self._setup_snap()
        metadata = {"invalid": "foo"}
        raised = self.assertRaises(
            errors.StoreMetadataError,
            self.client.push_metadata,
            "basic",
            metadata,
            False,
        )
        self.assertThat(str(raised), Equals("Received 400: 'Invalid field: invalid'"))

    def test_all_ok(self):
        self._setup_snap()
        metadata = {"field_ok": "foo"}
        result = self.client.push_metadata("basic", metadata, False)
        self.assertIsNone(result)

    def test_conflicting_simple_normal(self):
        self._setup_snap()
        metadata = {"test-conflict": "value"}
        raised = self.assertRaises(
            errors.StoreMetadataError,
            self.client.push_metadata,
            "basic",
            metadata,
            False,
        )
        should = """
            Metadata not pushed!
            Conflict in 'test-conflict' field:
                In snapcraft.yaml: 'value'
                In the Store:      'value-changed'
            You can repeat the push-metadata command with --force to force the local values into the Store
        """  # NOQA
        self.assertThat(str(raised), Equals(dedent(should).strip()))

    def test_conflicting_multiple_normal(self):
        self._setup_snap()
        metadata = {"test-conflict-1": "value-1", "test-conflict-2": "value-2"}
        raised = self.assertRaises(
            errors.StoreMetadataError,
            self.client.push_metadata,
            "basic",
            metadata,
            False,
        )
        should = """
            Metadata not pushed!
            Conflict in 'test-conflict-1' field:
                In snapcraft.yaml: 'value-1'
                In the Store:      'value-1-changed'
            Conflict in 'test-conflict-2' field:
                In snapcraft.yaml: 'value-2'
                In the Store:      'value-2-changed'
            You can repeat the push-metadata command with --force to force the local values into the Store
        """  # NOQA
        self.assertThat(str(raised), Equals(dedent(should).strip()))

    def test_conflicting_force(self):
        self._setup_snap()
        metadata = {"test-conflict": "value"}
        # force the update, even on conflicts!
        result = self.client.push_metadata("basic", metadata, True)
        self.assertIsNone(result)

    def test_braces_in_error_messages_are_literals(self):
        self._setup_snap()
        metadata = {"test-conflict-with-braces": "value"}
        raised = self.assertRaises(
            errors.StoreMetadataError,
            self.client.push_metadata,
            "basic",
            metadata,
            False,
        )
        should = """
            Metadata not pushed!
            Conflict in 'test-conflict-with-braces' field:
                In snapcraft.yaml: 'value'
                In the Store:      'value with {braces}'
            You can repeat the push-metadata command with --force to force the local values into the Store
        """  # NOQA
        self.assertThat(str(raised), Equals(dedent(should).strip()))


class PushBinaryMetadataTestCase(StoreTestCase):
    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)

    def _setup_snap(self):
        """Login, register and push a snap.

        These are all the previous steps needed to push binary metadata.
        """
        self.client.login("dummy", "test correct password")
        self.client.register("basic")
        path = os.path.join(os.path.dirname(tests.__file__), "data", "test-snap.snap")
        tracker = self.client.upload("basic", path)
        tracker.track()

    def test_requires_login(self):
        self.assertRaises(
            errors.InvalidCredentialsError,
            self.client.push_binary_metadata,
            "basic",
            {},
            False,
        )

    def test_refreshes_macaroon(self):
        self._setup_snap()
        self.fake_store.needs_refresh = True
        with tempfile.NamedTemporaryFile(suffix="ok") as f:
            metadata = {"icon": f}
            self.client.push_binary_metadata("basic", metadata, False)
        self.assertFalse(self.fake_store.needs_refresh)

    def test_invalid_data(self):
        self._setup_snap()
        with tempfile.NamedTemporaryFile(suffix="invalid") as f:
            metadata = {"icon": f}
            raised = self.assertRaises(
                errors.StoreMetadataError,
                self.client.push_binary_metadata,
                "basic",
                metadata,
                False,
            )
        self.assertThat(str(raised), Equals("Received 400: 'Invalid field: icon'"))

    def test_all_ok(self):
        self._setup_snap()
        with tempfile.NamedTemporaryFile(suffix="ok") as f:
            metadata = {"icon": f}
            result = self.client.push_binary_metadata("basic", metadata, False)
        self.assertIsNone(result)

    def test_conflicting_simple_normal(self):
        self._setup_snap()
        with tempfile.NamedTemporaryFile(suffix="conflict") as f:
            filename = os.path.basename(f.name)
            metadata = {"icon": f}
            raised = self.assertRaises(
                errors.StoreMetadataError,
                self.client.push_binary_metadata,
                "basic",
                metadata,
                False,
            )
        should = (
            """
            Metadata not pushed!
            Conflict in 'icon' field:
                In snapcraft.yaml: '{}'
                In the Store:      'original-icon'
            You can repeat the push-metadata command with --force to force the local values into the Store
        """
        ).format(
            filename
        )  # NOQA
        self.assertThat(str(raised), Equals(dedent(should).strip()))

    def test_conflicting_force(self):
        self._setup_snap()
        with tempfile.NamedTemporaryFile(suffix="conflict") as f:
            metadata = {"icon": f}
            # force the update, even on conflicts!
            result = self.client.push_binary_metadata("basic", metadata, True)
        self.assertIsNone(result)

    def test_braces_in_error_messages_are_literals(self):
        self._setup_snap()
        with tempfile.NamedTemporaryFile(suffix="conflict-with-braces") as f:
            filename = os.path.basename(f.name)
            metadata = {"icon": f}
            raised = self.assertRaises(
                errors.StoreMetadataError,
                self.client.push_binary_metadata,
                "basic",
                metadata,
                False,
            )
        should = """
            Metadata not pushed!
            Conflict in 'icon' field:
                In snapcraft.yaml: '{}'
                In the Store:      'original icon with {{braces}}'
            You can repeat the push-metadata command with --force to force the local values into the Store
        """.format(
            filename
        )  # NOQA
        self.assertThat(str(raised), Equals(dedent(should).strip()))


class SnapNotFoundTestCase(StoreTestCase):

    scenarios = (
        ("push_metadata", dict(attribute="push_metadata")),
        ("push_binary_metadata", dict(attribute="push_binary_metadata")),
    )

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)

    def _setup_snap(self):
        """Login, register and push a snap.

        These are all the previous steps needed to push binary metadata.
        """
        self.client.login("dummy", "test correct password")
        self.client.register("basic")
        path = os.path.join(os.path.dirname(tests.__file__), "data", "test-snap.snap")
        tracker = self.client.upload("basic", path)
        tracker.track()

    def test_snap_not_found(self):
        self._setup_snap()
        metadata = {"field_ok": "dummy"}
        raised = self.assertRaises(
            errors.SnapNotFoundError,
            getattr(self.client, self.attribute),
            "test-unexistent-snap",
            metadata,
            False,
        )
        self.assertThat(
            str(raised),
            Equals(
                "Snap 'test-unexistent-snap' was not "
                "found in '{}' series.".format(constants.DEFAULT_SERIES)
            ),
        )
