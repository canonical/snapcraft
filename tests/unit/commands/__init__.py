# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
import os
from textwrap import dedent

from click.testing import CliRunner

from snapcraft import storeapi
from snapcraft.cli._runner import run
from tests import fixture_setup, unit


_sample_keys = [
    {
        "name": "default",
        "sha3-384": (
            "vdEeQvRxmZ26npJCFaGnl-VfGz0lU2jZZkWp_s7E-RxVCNtH2_mtjcxq2NkDKkIp"
        ),
    },
    {
        "name": "another",
        "sha3-384": (
            "JsfToV5hO2eN9l89pYYCKXUioTERrZIIHUgQQd47jW8YNNBskupiIjWYd3KXLY_D"
        ),
    },
]


def get_sample_key(name):
    for key in _sample_keys:
        if key["name"] == name:
            return key
    raise KeyError(name)


def mock_snap_output(command, *args, **kwargs):
    if command == ["snap", "keys", "--json"]:
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
    else:
        raise AssertionError("Unhandled command: {}".format(command))


class CommandBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.runner = CliRunner()

    def run_command(self, args, **kwargs):
        return self.runner.invoke(run, args, catch_exceptions=False, **kwargs)


class LifecycleCommandsBaseTestCase(CommandBaseTestCase):

    yaml_template = """name: {step}-test
version: 1.0
summary: test {step}
description: if the {step} is successful the state file will be updated
confinement: strict
grade: stable

parts:
{parts}"""

    yaml_part = """  {step}{iter:d}:
    plugin: nil"""

    def make_snapcraft_yaml(self, step, n=1, yaml_part=None, create=False):
        if not yaml_part:
            yaml_part = self.yaml_part

        parts = "\n".join([yaml_part.format(step=step, iter=i) for i in range(n)])
        super().make_snapcraft_yaml(self.yaml_template.format(step=step, parts=parts))

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
