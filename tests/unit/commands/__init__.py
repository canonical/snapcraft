# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2019 Canonical Ltd
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

import fixtures
import json
import os
import subprocess
from textwrap import dedent

from click.testing import CliRunner

from snapcraft import storeapi
from snapcraft.cli._runner import run
from tests import fixture_setup, unit

_sample_keys = [
    {
        "name": "default",
        "sha3-384": "vdEeQvRxmZ26npJCFaGnl-VfGz0lU2jZZkWp_s7E-RxVCNtH2_mtjcxq2NkDKkIp",
    },
    {
        "name": "another",
        "sha3-384": "JsfToV5hO2eN9l89pYYCKXUioTERrZIIHUgQQd47jW8YNNBskupiIjWYd3KXLY_D",
    },
]


def get_sample_key(name):
    for key in _sample_keys:
        if key["name"] == name:
            return key
    raise KeyError(name)


original_check_output = subprocess.check_output


def mock_check_output(command, *args, **kwargs):
    if command[0].endswith("unsquashfs"):
        return original_check_output(command, *args, **kwargs)
    elif command == ["snap", "keys", "--json"]:
        return json.dumps(_sample_keys)
    elif command[:2] == ["snap", "export-key"]:
        if not command[2].startswith("--account="):
            raise AssertionError("Unhandled command: {}".format(command))
        account_id = command[2][len("--account=") :]
        name = command[3]
        # This isn't a full account-key-request assertion, but it's enough
        # for testing.
        return dedent(
            """\
            type: account-key-request
            account-id: {account_id}
            name: {name}
            public-key-sha3-384: {sha3_384}
            """
        ).format(
            account_id=account_id, name=name, sha3_384=get_sample_key(name)["sha3-384"]
        )
    elif command == ["snap", "create-key", "new-key"]:
        pass
    else:
        raise AssertionError("Unhandled command: {}".format(command))


class CommandBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.runner = CliRunner()

    def run_command(self, args, **kwargs):
        # For click testing, runner will overwrite the descriptors for stdio -
        # ensure TTY always appears connected.
        self.useFixture(
            fixtures.MockPatch("snapcraft.cli.echo.is_tty_connected", return_value=True)
        )

        return self.runner.invoke(run, args, catch_exceptions=False, **kwargs)


class LifecycleCommandsBaseTestCase(CommandBaseTestCase):

    yaml_template = """name: {step}-test
version: "1.0"
summary: test {step}
description: if the {step} is successful the state file will be updated
confinement: strict
grade: stable
base: {base}

parts:
{parts}"""

    yaml_part = """  {step}{iter:d}:
    plugin: nil"""

    def make_snapcraft_yaml(
        self, step, n=1, yaml_part=None, create=False, base="core18"
    ):
        if not yaml_part:
            yaml_part = self.yaml_part

        parts = "\n".join([yaml_part.format(step=step, iter=i) for i in range(n)])
        super().make_snapcraft_yaml(
            self.yaml_template.format(step=step, parts=parts, base=base)
        )

        parts = []
        for i in range(n):
            part_dir = os.path.join(self.parts_dir, "{}{}".format(step, i))
            state_dir = os.path.join(part_dir, "state")
            parts.append({"part_dir": part_dir, "state_dir": state_dir})

        return parts


class StoreCommandsBaseTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()
        self.fake_store = fixture_setup.FakeStore()
        self.useFixture(self.fake_store)
        self.client = storeapi.StoreClient()


class FakeStoreCommandsBaseTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

        self.fake_store_login = fixtures.MockPatchObject(storeapi.StoreClient, "login")
        self.useFixture(self.fake_store_login)

        self.fake_store_register = fixtures.MockPatchObject(
            storeapi._sca_client.SCAClient, "register"
        )
        self.useFixture(self.fake_store_register)

        self.fake_store_account_info = fixtures.MockPatchObject(
            storeapi._sca_client.SCAClient,
            "get_account_information",
            return_value={
                "account_id": "abcd",
                "account_keys": list(),
                "snaps": {
                    "16": {
                        "snap-test": {
                            "snap-id": "snap-test-snap-id",
                            "status": "Approved",
                            "private": False,
                            "since": "2016-12-12T01:01Z",
                            "price": "0",
                        }
                    }
                },
            },
        )
        self.useFixture(self.fake_store_account_info)

        self.fake_store_status = fixtures.MockPatchObject(
            storeapi._sca_client.SCAClient, "snap_status", return_value=dict()
        )
        self.useFixture(self.fake_store_status)

        self.fake_store_revisions = fixtures.MockPatchObject(
            storeapi._sca_client.SCAClient, "snap_revisions", return_value=dict()
        )
        self.useFixture(self.fake_store_revisions)

        self.fake_store_release = fixtures.MockPatchObject(
            storeapi.StoreClient, "release"
        )
        self.useFixture(self.fake_store_release)

        self.fake_store_register_key = fixtures.MockPatchObject(
            storeapi._sca_client.SCAClient, "register_key"
        )
        self.useFixture(self.fake_store_register_key)

        # Mock the snap command
        self.fake_check_output = fixtures.MockPatch(
            "subprocess.check_output", side_effect=mock_check_output
        )
        self.useFixture(self.fake_check_output)

        # Pretend that the snap command is available
        self.fake_package_installed = fixtures.MockPatch(
            "snapcraft.internal.repo.Repo.is_package_installed", return_value=True
        )
        self.useFixture(self.fake_package_installed)
