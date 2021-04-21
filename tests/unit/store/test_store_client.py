# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2021 Canonical Ltd
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
import pathlib
import tempfile
from textwrap import dedent
from unittest import mock

import fixtures
from testtools.matchers import (
    Contains,
    Equals,
    FileExists,
    HasLength,
    Is,
    IsInstance,
    Not,
)

import tests
from snapcraft import storeapi
from snapcraft.storeapi import errors, http_clients
from snapcraft.storeapi.v2 import channel_map, releases, whoami, validation_sets
from tests import fixture_setup, unit


class StoreTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_store = self.useFixture(fixture_setup.FakeStore())
        self.client = storeapi.StoreClient()


class LoginTestCase(StoreTestCase):
    def test_login_successful(self):
        self.client.login(email="dummy email", password="test correct password")
        self.assertIsNotNone(self.client.auth_client.auth)

    def test_login_successful_with_one_time_password(self):
        self.client.login(
            email="dummy email",
            password="test correct password",
            otp="test correct one-time password",
        )
        self.assertIsNotNone(self.client.auth_client.auth)

    def test_login_successful_with_package_attenuation(self):
        self.client.login(
            email="dummy email",
            password="test correct password",
            packages=[{"name": "foo", "series": "16"}],
        )
        self.assertIsNotNone(self.client.auth_client.auth)

    def test_login_successful_with_channel_attenuation(self):
        self.client.login(
            email="dummy email", password="test correct password", channels=["edge"]
        )
        self.assertIsNotNone(self.client.auth_client.auth)

    def test_login_successful_fully_attenuated(self):
        self.client.login(
            email="dummy email",
            password="test correct password",
            packages=[{"name": "foo", "series": "16"}],
            channels=["edge"],
            save=False,
        )
        # Client configuration is filled, but it's not saved on disk.
        self.assertThat(
            self.client.auth_client._conf._get_config_path(), Not(FileExists())
        )
        self.assertIsNotNone(self.client.auth_client.auth)

    def test_login_successful_with_expiration(self):
        self.client.login(
            email="dummy email",
            password="test correct password",
            packages=[{"name": "foo", "series": "16"}],
            channels=["edge"],
            expires="2017-12-22",
        )
        self.assertIsNotNone(self.client.auth_client.auth)

    def test_login_with_exported_login(self):
        with pathlib.Path("test-exported-login").open("w") as config_fd:
            print(
                "[{}]".format(self.client.auth_client._conf._get_section_name()),
                file=config_fd,
            )
            print(
                "macaroon=MDAwZWxvY2F0aW9uIAowMDEwaWRlbnRpZmllciAKMDAxNGNpZCB0ZXN0IGNhdmVhdAowMDE5dmlkIHRlc3QgdmVyaWZpYWNpb24KMDAxN2NsIGxvY2FsaG9zdDozNTM1MQowMDBmc2lnbmF0dXJlIAo",
                file=config_fd,
            )
            print(
                "unbound_discharge=MDAwZWxvY2F0aW9uIAowMDEwaWRlbnRpZmllciAKMDAwZnNpZ25hdHVyZSAK",
                file=config_fd,
            )
            config_fd.flush()

        with pathlib.Path("test-exported-login").open() as config_fd:
            self.client.login(config_fd=config_fd)

        self.assertThat(
            self.client.auth_client._conf.get("macaroon"),
            Equals(
                "MDAwZWxvY2F0aW9uIAowMDEwaWRlbnRpZmllciAKMDAxNGNpZCB0ZXN0IGNhdmVhdAowMDE5dmlkIHRlc3QgdmVyaWZpYWNpb24KMDAxN2NsIGxvY2FsaG9zdDozNTM1MQowMDBmc2lnbmF0dXJlIAo"
            ),
        )
        self.assertThat(
            self.client.auth_client._conf.get("unbound_discharge"),
            Equals("MDAwZWxvY2F0aW9uIAowMDEwaWRlbnRpZmllciAKMDAwZnNpZ25hdHVyZSAK"),
        )
        self.assertThat(
            self.client.auth_client.auth,
            Equals(
                "Macaroon root=MDAwZWxvY2F0aW9uIAowMDEwaWRlbnRpZmllciAKMDAxNGNpZCB0ZXN0IGNhdmVhdAowMDE5dmlkIHRlc3QgdmVyaWZpYWNpb24KMDAxN2NsIGxvY2FsaG9zdDozNTM1MQowMDBmc2lnbmF0dXJlIAo, discharge=MDAwZWxvY2F0aW9uIAowMDEwaWRlbnRpZmllciAKMDAyZnNpZ25hdHVyZSDmRizXTOkAmfmy5hGCm7F0H4LBea16YbJYVhDkAJZ-Ago"
            ),
        )

    def test_failed_login_with_wrong_password(self):
        self.assertRaises(
            http_clients.errors.StoreAuthenticationError,
            self.client.login,
            email="dummy email",
            password="wrong password",
        )

    def test_failed_login_requires_one_time_password(self):
        self.assertRaises(
            http_clients.errors.StoreTwoFactorAuthenticationRequired,
            self.client.login,
            email="dummy email",
            password="test requires 2fa",
        )

    def test_failed_login_with_wrong_one_time_password(self):
        self.assertRaises(
            http_clients.errors.StoreAuthenticationError,
            self.client.login,
            email="dummy email",
            password="test correct password",
            otp="wrong one-time password",
        )

    def test_failed_login_with_unregistered_snap(self):
        raised = self.assertRaises(
            errors.GeneralStoreError,
            self.client.login,
            email="dummy email",
            password="test correct password",
            packages=[{"name": "unregistered-snap-name", "series": "16"}],
        )

        self.assertThat(str(raised), Contains("not found"))


class DownloadTestCase(StoreTestCase):

    # sha3-384 of tests/data/test-snap.snap
    EXPECTED_SHA3_384 = ""

    def test_download_nonexistent_snap_raises_exception(self):
        self.client.login(email="dummy", password="test correct password")

        raised = self.assertRaises(
            errors.SnapNotFoundError,
            self.client.download,
            "nonexistent-snap",
            risk="stable",
            download_path="dummy.snap",
            arch="test-arch",
        )

        self.expectThat(raised._snap_name, Equals("nonexistent-snap"))
        self.expectThat(raised._channel, Is(None))
        self.expectThat(raised._arch, Is(None))

    def test_download_snap(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.client.login(email="dummy", password="test correct password")
        download_path = os.path.join(self.path, "test-snap.snap")
        self.client.download("test-snap", risk="stable", download_path=download_path)
        self.assertThat(download_path, FileExists())

    def test_download_snap_missing_risk(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.client.login(email="dummy", password="test correct password")

        raised = self.assertRaises(
            errors.SnapNotFoundError,
            self.client.download,
            "test-snap",
            risk="beta",
            download_path="dummy.snap",
        )

        self.expectThat(raised._snap_name, Equals("test-snap"))
        self.expectThat(raised._channel, Equals("beta"))
        self.expectThat(raised._arch, Is(None))

    def test_download_from_brand_store_requires_store(self):
        self.client.login(email="dummy", password="test correct password")
        raised = self.assertRaises(
            errors.SnapNotFoundError,
            self.client.download,
            "test-snap-brand-store",
            risk="stable",
            download_path="dummy.snap",
        )

        self.expectThat(raised._snap_name, Equals("test-snap-brand-store"))
        self.expectThat(raised._channel, Is(None))
        self.expectThat(raised._arch, Is(None))

    def test_download_from_branded_store(self):
        # Downloading from a branded-store requires setting the
        # 'SNAPCRAFT_UBUNTU_STORE' environment variable to the
        # correct store 'slug' (the branded store identifier).
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_UBUNTU_STORE", "Test-Branded")
        )
        self.client.login(email="dummy", password="test correct password")

        download_path = os.path.join(self.path, "brand.snap")
        self.client.download(
            "test-snap-branded-store", risk="stable", download_path=download_path
        )
        self.assertThat(download_path, FileExists())

    def test_download_already_downloaded_snap(self):
        self.client.login(email="dummy", password="test correct password")
        download_path = os.path.join(self.path, "test-snap.snap")
        # download first time.
        self.client.download("test-snap", risk="stable", download_path=download_path)
        first_stat = os.stat(download_path)
        # download again.
        self.client.download("test-snap", risk="stable", download_path=download_path)
        second_stat = os.stat(download_path)
        # If these are equal it means a second download did not happen.
        self.assertThat(second_stat.st_ctime, Equals(first_stat.st_ctime))

    def test_download_on_sha_mismatch(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)
        self.client.login(email="dummy", password="test correct password")
        download_path = os.path.join(self.path, "test-snap.snap")
        # Write a wrong file in the download path.
        open(download_path, "w").close()
        first_stat = os.stat(download_path)
        self.client.download("test-snap", risk="stable", download_path=download_path)
        second_stat = os.stat(download_path)
        # If these are different it means that the download did happen.
        self.assertThat(second_stat, Not(Equals(first_stat)))

    def test_download_with_hash_mismatch_raises_exception(self):
        self.client.login(email="dummy", password="test correct password")
        download_path = os.path.join(self.path, "test-snap.snap")
        self.assertRaises(
            errors.SHAMismatchError,
            self.client.download,
            "test-snap-with-wrong-sha",
            risk="stable",
            download_path=download_path,
        )


class PushSnapBuildTestCase(StoreTestCase):
    def test_push_snap_build_refreshes_macaroon(self):
        self.client.login(email="dummy", password="test correct password")
        self.fake_store.needs_refresh = True
        self.client.push_snap_build("snap-id", "dummy")
        self.assertFalse(self.fake_store.needs_refresh)

    def test_push_snap_build_not_implemented(self):
        # If the "enable_snap_build" feature switch is off in the store, we
        # will get a descriptive error message.
        self.client.login(email="dummy", password="test correct password")
        raised = self.assertRaises(
            http_clients.errors.StoreServerError,
            self.client.push_snap_build,
            "snap-id",
            "test-not-implemented",
        )
        self.assertThat(raised.error_code, Equals(501))

    def test_push_snap_build_invalid_data(self):
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
        raised = self.assertRaises(
            http_clients.errors.StoreServerError,
            self.client.push_snap_build,
            "snap-id",
            "test-unexpected-data",
        )
        self.assertThat(raised.error_code, Equals(500))

    def test_push_snap_build_successfully(self):
        self.client.login(email="dummy", password="test correct password")
        # No exception will be raised if this is successful.
        self.client.push_snap_build("snap-id", "dummy")


class GetAccountInformationTestCase(StoreTestCase):
    def test_get_account_information_successfully(self):
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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
    def test_register_key_successfully(self):
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
        raised = self.assertRaises(
            http_clients.errors.StoreServerError,
            self.client.register_key,
            "test-not-implemented",
        )
        self.assertThat(raised.error_code, Equals(501))

    def test_invalid_data(self):
        self.client.login(email="dummy", password="test correct password")
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
    def test_register_name_successfully(self):
        self.client.login(email="dummy", password="test correct password")
        # No exception will be raised if this is successful
        self.client.register("test-good-snap-name")

    def test_register_name_successfully_to_store_id(self):
        self.client.login(email="dummy", password="test correct password")
        # No exception will be raised if this is successful
        self.client.register("test-good-snap-name", store_id="my-brand")

    def test_register_private_name_successfully(self):
        self.client.login(email="dummy", password="test correct password")
        # No exception will be raised if this is successful
        self.client.register("test-good-snap-name", is_private=True)

    def test_register_refreshes_macaroon(self):
        self.client.login(email="dummy", password="test correct password")
        self.fake_store.needs_refresh = True
        self.client.register("test-good-snap-name")
        self.assertFalse(self.fake_store.needs_refresh)

    def test_already_registered(self):
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
        raised = self.assertRaises(
            errors.StoreRegistrationError,
            self.client.register,
            "snap-name-no-clear-error",
        )
        self.assertThat(str(raised), Equals("Registration failed."))


class ValidationSetsTestCase(StoreTestCase):
    def setUp(self):
        super().setUp()
        self.client.login(email="dummy", password="test correct password")

        self.validation_sets_build = {
            "name": "acme-cert-2020-10",
            "account-id": "AccountIDXXXOfTheRequestingUserX",
            "sequence": 3,
            "snaps": [
                {"name": "snap-name-1", "presence": "optional"},
                {"name": "snap-name-2"},
                {
                    "name": "snap-name-3",
                    "id": "XXSnapIDForXSnapName3XXXXXXXXXXX",
                    "presence": "required",
                    "revision": 2,
                },
                {"name": "snap-name-4", "revision": 123},
                {"name": "snap-name-5", "presence": "invalid"},
            ],
        }

    def test_post_valid_build_assertion(self):
        build_assertion = self.client.post_validation_sets_build_assertion(
            validation_sets=self.validation_sets_build
        )

        self.assertThat(build_assertion, IsInstance(validation_sets.BuildAssertion))

    def test_post_invalid_sequence_for_build_assertion(self):
        self.validation_sets_build["sequence"] = 0

        raised = self.assertRaises(
            errors.StoreValidationSetsError,
            self.client.post_validation_sets_build_assertion,
            validation_sets=self.validation_sets_build,
        )

        self.assertThat(
            str(raised),
            Equals(
                "Issues encountered with validation set: 0 is less than the minimum of 1 at /sequence"
            ),
        )

    def _fake_sign(self, build_assertion: validation_sets.BuildAssertion) -> bytes:
        # Fake sign.
        assertion_json = build_assertion.marshal()
        assertion_json[
            "sign-key-sha3-384"
        ] = "XSignXKeyXHashXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
        return (json.dumps(assertion_json) + "\n\nSIGNED").encode()

    def test_post_valid_assertion(self):
        build_assertion = self.client.post_validation_sets_build_assertion(
            validation_sets=self.validation_sets_build
        )

        vs = self.client.post_validation_sets(
            signed_validation_sets=self._fake_sign(build_assertion)
        )

        self.assertThat(vs, IsInstance(validation_sets.ValidationSets))
        self.assertThat(vs.assertions, HasLength(1))
        self.assertThat(vs.assertions[0], Equals(build_assertion))

    def test_post_invalid_assertion(self):
        build_assertion = self.client.post_validation_sets_build_assertion(
            validation_sets=self.validation_sets_build
        )
        build_assertion.authority_id = "invalid"

        raised = self.assertRaises(
            errors.StoreValidationSetsError,
            self.client.post_validation_sets,
            signed_validation_sets=self._fake_sign(build_assertion),
        )

        self.assertThat(
            str(raised),
            Equals(
                "Issues encountered with validation set: account-id and authority-id must match the requesting user."
            ),
        )

    def test_get_validation_sets_empty(self):
        vs = self.client.get_validation_sets()

        self.assertThat(vs, IsInstance(validation_sets.ValidationSets))
        self.assertThat(vs.assertions, HasLength(0))

    def test_get_validation_sets(self):
        build_assertion = self.client.post_validation_sets_build_assertion(
            validation_sets=self.validation_sets_build
        )
        vs_1 = self.client.post_validation_sets(
            signed_validation_sets=self._fake_sign(build_assertion)
        )

        vs = self.client.get_validation_sets()

        self.assertThat(vs, IsInstance(validation_sets.ValidationSets))
        self.assertThat(vs.assertions, HasLength(1))
        self.expectThat(vs.assertions, Contains(vs_1.assertions[0]))

    def test_get_multiple_validation_sets(self):
        build_assertion = self.client.post_validation_sets_build_assertion(
            validation_sets=self.validation_sets_build
        )
        vs_1 = self.client.post_validation_sets(
            signed_validation_sets=self._fake_sign(build_assertion)
        )

        # Create a different build_assertion from the first one.
        build_assertion.name = "not-acme"
        vs_2 = self.client.post_validation_sets(
            signed_validation_sets=self._fake_sign(build_assertion)
        )

        vs = self.client.get_validation_sets()

        self.assertThat(vs, IsInstance(validation_sets.ValidationSets))
        self.expectThat(vs.assertions, Contains(vs_1.assertions[0]))
        self.expectThat(vs.assertions, Contains(vs_2.assertions[0]))

    def test_get_validation_sets_by_name(self):
        build_assertion = self.client.post_validation_sets_build_assertion(
            validation_sets=self.validation_sets_build
        )
        self.client.post_validation_sets(
            signed_validation_sets=self._fake_sign(build_assertion)
        )

        # Create a different build_assertion from the first one.
        build_assertion.name = "not-acme"
        self.client.post_validation_sets(
            signed_validation_sets=self._fake_sign(build_assertion)
        )
        vs = self.client.get_validation_sets(name="acme-cert-2020-10")

        self.assertThat(vs, IsInstance(validation_sets.ValidationSets))
        self.expectThat(vs.assertions, HasLength(1))
        self.expectThat(vs.assertions[0].name, Equals("acme-cert-2020-10"))

    def test_get_invalid_sequence(self):
        raised = self.assertRaises(
            errors.StoreValidationSetsError,
            self.client.get_validation_sets,
            sequence="invalid",
        )

        self.assertThat(
            str(raised),
            Equals(
                "Issues encountered with validation set: sequence must be one of "
                "['all', 'latest'] or a positive integer higher than 0."
            ),
        )


class ValidationsTestCase(StoreTestCase):
    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)

    def test_get_success(self):
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")

        err = self.assertRaises(
            errors.StoreValidationError, self.client.get_assertion, "bad", "validations"
        )

        expected = "Received error 200: 'Invalid response from the server'"
        self.assertThat(str(err), Equals(expected))
        self.assertIn("Invalid response from the server", self.fake_logger.output)

    def test_get_error_response(self):
        self.client.login(email="dummy", password="test correct password")

        err = self.assertRaises(
            http_clients.errors.StoreNetworkError,
            self.client.get_assertion,
            "err",
            "validations",
        )

        expected = "maximum retries exceeded"
        self.assertThat(str(err), Contains(expected))

    def test_push_success(self):
        self.client.login(email="dummy", password="test correct password")
        assertion = json.dumps({"foo": "bar"}).encode("utf-8")

        result = self.client.push_assertion("good", assertion, "validations")

        expected = {"assertion": '{"foo": "bar"}'}
        self.assertThat(result, Equals(expected))

    def test_push_bad_response(self):
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
        assertion = json.dumps({"foo": "bar"}).encode("utf-8")

        err = self.assertRaises(
            http_clients.errors.StoreServerError,
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

    def test_upload_snap(self):
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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

        self.client.login(email="dummy", password="test correct password")

        raised = self.assertRaises(
            http_clients.errors.StoreServerError,
            self.client.upload,
            "test-snap",
            self.snap_path,
        )
        self.assertThat(raised.error_code, Equals(500))

    def test_upload_snap_requires_review(self):
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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

    def test_upload_unregistered_snap(self):
        self.client.login(email="dummy", password="test correct password")
        raised = self.assertRaises(
            errors.StoreUploadError,
            self.client.upload,
            "test-snap-unregistered",
            self.snap_path,
        )
        self.assertThat(
            str(raised),
            Equals("This snap is not registered. Register the snap and try again."),
        )

    def test_upload_forbidden_snap(self):
        self.client.login(email="dummy", password="test correct password")
        raised = self.assertRaises(
            errors.StoreUploadError,
            self.client.upload,
            "test-snap-forbidden",
            self.snap_path,
        )
        self.assertThat(
            str(raised),
            Equals(
                "You are not the publisher or allowed to upload revisions for "
                "this snap. Ensure you are logged in with the proper account "
                "and try again."
            ),
        )


class ReleaseTest(StoreTestCase):
    def test_release_snap(self):
        self.client.login(email="dummy", password="test correct password")
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

    def test_progressive_release_snap(self):
        self.client.login(email="dummy", password="test correct password")
        channel_map = self.client.release(
            "test-snap", "19", ["beta"], progressive_percentage=10
        )
        expected_channel_map = {
            "opened_channels": ["beta"],
            "channel_map": [
                {"channel": "stable", "info": "none"},
                {"channel": "candidate", "info": "none"},
                {"revision": 19, "channel": "beta", "version": "0", "info": "specific"},
                {"channel": "edge", "info": "tracking"},
            ],
        }
        # The channel map would be the same as if no progressive release was
        # done.
        self.assertThat(channel_map, Equals(expected_channel_map))

    def test_release_refreshes_macaroon(self):
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
        raised = self.assertRaises(
            errors.StoreReleaseError, self.client.release, "test-snap", "19", ["alpha"]
        )

        self.assertThat(str(raised), Equals("Not a valid channel: alpha"))

    def test_release_snap_to_bad_channel(self):
        self.client.login(email="dummy", password="test correct password")
        self.assertRaises(
            http_clients.errors.StoreServerError,
            self.client.release,
            "test-snap",
            "19",
            ["bad-channel"],
        )

    def test_release_unregistered_snap(self):
        self.client.login(email="dummy", password="test correct password")
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

    def test_release_with_invalid_revision(self):
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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

    def test_close_refreshes_macaroon(self):
        self.client.login(email="dummy", password="test correct password")
        self.fake_store.needs_refresh = True
        self.client.close_channels("snap-id", ["dummy"])
        self.assertFalse(self.fake_store.needs_refresh)

    def test_close_invalid_data(self):
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
        raised = self.assertRaises(
            http_clients.errors.StoreServerError,
            self.client.close_channels,
            "snap-id",
            ["unexpected"],
        )
        self.assertThat(raised.error_code, Equals(500))

    def test_close_broken_store_plain(self):
        # If the contract is broken by the Store, users will be have additional
        # debug information available.
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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


class GetSnapStatusTestCase(StoreTestCase):
    def setUp(self):
        super().setUp()
        self.expected = {
            "channel_map_tree": {
                "latest": {
                    "16": {
                        "i386": [
                            {"channel": "stable", "info": "none"},
                            {"channel": "candidate", "info": "none"},
                            {
                                "channel": "beta",
                                "info": "specific",
                                "revision": 6,
                                "version": "1.1-amd64",
                            },
                            {
                                "channel": "edge",
                                "info": "specific",
                                "revision": 3,
                                "version": "1.0-i386",
                            },
                            {
                                "channel": "edge/test",
                                "info": "branch",
                                "revision": 9,
                                "version": "1.1-i386",
                                "expires_at": "2019-05-30T01:17:06.465504",
                            },
                        ],
                        "all": [
                            {"channel": "stable", "info": "none"},
                            {"channel": "candidate", "info": "none"},
                            {
                                "channel": "beta",
                                "info": "specific",
                                "revision": 6,
                                "version": "1.1-amd64",
                            },
                            {
                                "channel": "edge",
                                "info": "specific",
                                "revision": 3,
                                "version": "1.0-i386",
                            },
                            {
                                "channel": "edge/test",
                                "info": "branch",
                                "revision": 9,
                                "version": "1.1-i386",
                                "expires_at": "2019-05-30T01:17:06.465504",
                            },
                        ],
                        "amd64": [
                            {
                                "channel": "stable",
                                "info": "specific",
                                "revision": 2,
                                "version": "1.0-amd64",
                            },
                            {"channel": "candidate", "info": "none"},
                            {
                                "channel": "beta",
                                "info": "specific",
                                "revision": 4,
                                "version": "1.1-amd64",
                            },
                            {"channel": "edge", "info": "tracking"},
                            {
                                "channel": "edge/test",
                                "info": "branch",
                                "revision": 10,
                                "version": "1.1-amd64",
                                "expires_at": "2019-05-30T01:17:06.465504",
                            },
                        ],
                    }
                }
            }
        }

    def test_get_snap_status_successfully(self):
        self.client.login(email="dummy", password="test correct password")
        self.assertThat(self.client.get_snap_status("basic"), Equals(self.expected))

    def test_get_snap_status_filter_by_arch(self):
        self.client.login(email="dummy", password="test correct password")
        exp_arch = self.expected["channel_map_tree"]["latest"]["16"]["amd64"]
        self.assertThat(
            self.client.get_snap_status("basic", arch="amd64"),
            Equals({"channel_map_tree": {"latest": {"16": {"amd64": exp_arch}}}}),
        )

    def test_get_snap_status_filter_by_unknown_arch(self):
        self.client.login(email="dummy", password="test correct password")

        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.client.get_snap_status,
            "basic",
            arch="some-arch",
        )

        self.expectThat(raised._snap_name, Equals("basic"))
        self.expectThat(raised._channel, Is(None))
        self.expectThat(raised._arch, Is("some-arch"))

    def test_get_snap_status_no_id(self):
        self.client.login(email="dummy", password="test correct password")
        e = self.assertRaises(
            storeapi.errors.NoSnapIdError, self.client.get_snap_status, "no-id"
        )
        self.assertThat(e.snap_name, Equals("no-id"))

    def test_get_snap_status_refreshes_macaroon(self):
        self.client.login(email="dummy", password="test correct password")
        self.fake_store.needs_refresh = True
        self.assertThat(self.client.get_snap_status("basic"), Equals(self.expected))
        self.assertFalse(self.fake_store.needs_refresh)

    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    @mock.patch.object(storeapi._dashboard_api.DashboardAPI, "get")
    def test_get_snap_status_server_error(self, mock_sca_get, mock_account_info):
        mock_account_info.return_value = {
            "snaps": {"16": {"basic": {"snap-id": "my_snap_id"}}}
        }

        mock_sca_get.return_value = mock.Mock(
            ok=False, status_code=500, reason="Server error", json=lambda: {}
        )

        self.client.login(email="dummy", password="test correct password")
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


class SnapChannelMapTest(StoreTestCase):
    def test_get_snap_channel_map(self):
        self.client.login(email="dummy", password="test correct password")
        self.assertThat(
            self.client.get_snap_channel_map(snap_name="basic"),
            IsInstance(channel_map.ChannelMap),
        )


class SnapReleasesTest(StoreTestCase):
    def test_get_snap_releases(self):
        self.client.login(email="dummy", password="test correct password")
        self.assertThat(
            self.client.get_snap_releases(snap_name="basic"),
            IsInstance(releases.Releases),
        )


class WhoAmITest(StoreTestCase):
    def test_whoami(self):
        self.client.login(email="dummy", password="test correct password")
        self.assertThat(
            self.client.whoami(), IsInstance(whoami.WhoAmI),
        )


class SignDeveloperAgreementTestCase(StoreTestCase):
    def test_sign_dev_agreement_success(self):
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
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
        self.client.login(email="dummy", password="test correct password")
        raised = self.assertRaises(
            http_clients.errors.StoreServerError,
            self.client.sign_developer_agreement,
            latest_tos_accepted=True,
        )
        self.assertThat(raised.error_code, Equals(500))


class UploadMetadataTestCase(StoreTestCase):
    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)

    def _setup_snap(self):
        """Login, register and upload a snap.

        These are all the previous steps needed to upload metadata.
        """
        self.client.login(email="dummy", password="test correct password")
        self.client.register("basic")
        path = os.path.join(os.path.dirname(tests.__file__), "data", "test-snap.snap")
        tracker = self.client.upload("basic", path)
        tracker.track()

    def test_refreshes_macaroon(self):
        self._setup_snap()
        self.fake_store.needs_refresh = True
        metadata = {"field_ok": "foo"}
        self.client.upload_metadata("basic", metadata, False)
        self.assertFalse(self.fake_store.needs_refresh)

    def test_invalid_data(self):
        self._setup_snap()
        metadata = {"invalid": "foo"}
        raised = self.assertRaises(
            errors.StoreMetadataError,
            self.client.upload_metadata,
            "basic",
            metadata,
            False,
        )
        self.assertThat(str(raised), Equals("Received 400: 'Invalid field: invalid'"))

    def test_all_ok(self):
        self._setup_snap()
        metadata = {"field_ok": "foo"}
        result = self.client.upload_metadata("basic", metadata, False)
        self.assertIsNone(result)

    def test_conflicting_simple_normal(self):
        self._setup_snap()
        metadata = {"test-conflict": "value"}
        raised = self.assertRaises(
            errors.StoreMetadataError,
            self.client.upload_metadata,
            "basic",
            metadata,
            False,
        )
        should = """
            Metadata not uploaded!
            Conflict in 'test-conflict' field:
                In snapcraft.yaml: 'value'
                In the Store:      'value-changed'
            You can repeat the upload-metadata command with --force to force the local values into the Store
        """  # NOQA
        self.assertThat(str(raised), Equals(dedent(should).strip()))

    def test_conflicting_multiple_normal(self):
        self._setup_snap()
        metadata = {"test-conflict-1": "value-1", "test-conflict-2": "value-2"}
        raised = self.assertRaises(
            errors.StoreMetadataError,
            self.client.upload_metadata,
            "basic",
            metadata,
            False,
        )
        should = """
            Metadata not uploaded!
            Conflict in 'test-conflict-1' field:
                In snapcraft.yaml: 'value-1'
                In the Store:      'value-1-changed'
            Conflict in 'test-conflict-2' field:
                In snapcraft.yaml: 'value-2'
                In the Store:      'value-2-changed'
            You can repeat the upload-metadata command with --force to force the local values into the Store
        """  # NOQA
        self.assertThat(str(raised), Equals(dedent(should).strip()))

    def test_conflicting_force(self):
        self._setup_snap()
        metadata = {"test-conflict": "value"}
        # force the update, even on conflicts!
        result = self.client.upload_metadata("basic", metadata, True)
        self.assertIsNone(result)

    def test_braces_in_error_messages_are_literals(self):
        self._setup_snap()
        metadata = {"test-conflict-with-braces": "value"}
        raised = self.assertRaises(
            errors.StoreMetadataError,
            self.client.upload_metadata,
            "basic",
            metadata,
            False,
        )
        should = """
            Metadata not uploaded!
            Conflict in 'test-conflict-with-braces' field:
                In snapcraft.yaml: 'value'
                In the Store:      'value with {braces}'
            You can repeat the upload-metadata command with --force to force the local values into the Store
        """  # NOQA
        self.assertThat(str(raised), Equals(dedent(should).strip()))


class UploadBinaryMetadataTestCase(StoreTestCase):
    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)

    def _setup_snap(self):
        """Login, register and upload a snap.

        These are all the previous steps needed to upload binary metadata.
        """
        self.client.login(email="dummy", password="test correct password")
        self.client.register("basic")
        path = os.path.join(os.path.dirname(tests.__file__), "data", "test-snap.snap")
        tracker = self.client.upload("basic", path)
        tracker.track()

    def test_refreshes_macaroon(self):
        self._setup_snap()
        self.fake_store.needs_refresh = True
        with tempfile.NamedTemporaryFile(suffix="ok") as f:
            metadata = {"icon": f}
            self.client.upload_binary_metadata("basic", metadata, False)
        self.assertFalse(self.fake_store.needs_refresh)

    def test_invalid_data(self):
        self._setup_snap()
        with tempfile.NamedTemporaryFile(suffix="invalid") as f:
            metadata = {"icon": f}
            raised = self.assertRaises(
                errors.StoreMetadataError,
                self.client.upload_binary_metadata,
                "basic",
                metadata,
                False,
            )
        self.assertThat(str(raised), Equals("Received 400: 'Invalid field: icon'"))

    def test_all_ok(self):
        self._setup_snap()
        with tempfile.NamedTemporaryFile(suffix="ok") as f:
            metadata = {"icon": f}
            result = self.client.upload_binary_metadata("basic", metadata, False)
        self.assertIsNone(result)

    def test_conflicting_simple_normal(self):
        self._setup_snap()
        with tempfile.NamedTemporaryFile(suffix="conflict") as f:
            filename = os.path.basename(f.name)
            metadata = {"icon": f}
            raised = self.assertRaises(
                errors.StoreMetadataError,
                self.client.upload_binary_metadata,
                "basic",
                metadata,
                False,
            )
        should = """
            Metadata not uploaded!
            Conflict in 'icon' field:
                In snapcraft.yaml: '{}'
                In the Store:      'original-icon'
            You can repeat the upload-metadata command with --force to force the local values into the Store
        """.format(
            filename
        )  # NOQA
        self.assertThat(str(raised), Equals(dedent(should).strip()))

    def test_conflicting_force(self):
        self._setup_snap()
        with tempfile.NamedTemporaryFile(suffix="conflict") as f:
            metadata = {"icon": f}
            # force the update, even on conflicts!
            result = self.client.upload_binary_metadata("basic", metadata, True)
        self.assertIsNone(result)

    def test_braces_in_error_messages_are_literals(self):
        self._setup_snap()
        with tempfile.NamedTemporaryFile(suffix="conflict-with-braces") as f:
            filename = os.path.basename(f.name)
            metadata = {"icon": f}
            raised = self.assertRaises(
                errors.StoreMetadataError,
                self.client.upload_binary_metadata,
                "basic",
                metadata,
                False,
            )
        should = """
            Metadata not uploaded!
            Conflict in 'icon' field:
                In snapcraft.yaml: '{}'
                In the Store:      'original icon with {{braces}}'
            You can repeat the upload-metadata command with --force to force the local values into the Store
        """.format(
            filename
        )  # NOQA
        self.assertThat(str(raised), Equals(dedent(should).strip()))


class SnapNotFoundTestCase(StoreTestCase):
    def _setup_snap(self):
        """Login, register and upload a snap.

        These are all the previous steps needed to upload binary metadata.
        """
        self.client.login(email="dummy", password="test correct password")
        self.client.register("basic")
        path = os.path.join(os.path.dirname(tests.__file__), "data", "test-snap.snap")
        tracker = self.client.upload("basic", path)
        tracker.track()

    def assert_raises(self, method):
        self._setup_snap()

        metadata = {"field_ok": "dummy"}

        raised = self.assertRaises(
            errors.SnapNotFoundError,
            getattr(self.client, method),
            "test-nonexistent-snap",
            metadata,
            False,
        )

        self.expectThat(raised._snap_name, Equals("test-nonexistent-snap"))
        self.expectThat(raised._channel, Is(None))
        self.expectThat(raised._arch, Is(None))

    def test_snap_not_found_upload_metadata(self):
        self.assert_raises("upload_metadata")

    def test_snap_not_found_upload_binary_metadata(self):
        self.assert_raises("upload_binary_metadata")
