# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

import logging
import subprocess
from textwrap import dedent
from unittest import mock

import fixtures
import yaml
from testtools.matchers import Contains, Equals

from snapcraft import storeapi
from snapcraft.integrations import travis
from snapcraft.internal.errors import (
    RequiredCommandFailure,
    RequiredCommandNotFound,
    RequiredPathDoesNotExist,
)
from snapcraft.project import Project
import tests
from tests import unit


test_snapcraft_yaml = """
name: foo
version: 1
summary: a summary
description: a description
confinement: strict
grade: stable
parts:
  foo:
    plugin: nil
"""


class TravisPreconditionsTestCase(unit.TestCase):

    scenarios = [
        ("enable", {"entry_point": "enable"}),
        ("refresh", {"entry_point": "refresh"}),
    ]

    def run_command(self, cmd):
        getattr(travis, cmd)()

    @mock.patch("subprocess.check_call")
    def test_missing_travis_cli(self, mock_check_call):
        mock_check_call.side_effect = [FileNotFoundError()]

        raised = self.assertRaises(
            RequiredCommandNotFound, self.run_command, self.entry_point
        )

        self.assertThat(
            str(raised).splitlines(),
            Equals(
                [
                    "Travis CLI (`travis`) is not available.",
                    "Please install it before trying this command again:",
                    "",
                    "    $ sudo apt install ruby-dev ruby-ffi libffi-dev",
                    "    $ sudo gem install travis",
                ]
            ),
        )

    @mock.patch("subprocess.check_call")
    def test_broken_travis_cli(self, mock_check_call):
        mock_check_call.side_effect = [subprocess.CalledProcessError(1, "test")]

        raised = self.assertRaises(
            RequiredCommandFailure, self.run_command, self.entry_point
        )

        self.assertThat(
            str(raised).splitlines(),
            Equals(
                [
                    "Travis CLI (`travis settings`) is not functional or you are "
                    "not allowed to access this repository settings.",
                    "Make sure it works correctly in your system before trying "
                    "this command again.",
                ]
            ),
        )

    @mock.patch("subprocess.check_call")
    def test_missing_git(self, mock_check_call):
        mock_check_call.side_effect = [None, FileNotFoundError()]

        raised = self.assertRaises(
            RequiredCommandNotFound, self.run_command, self.entry_point
        )

        self.assertThat(
            str(raised).splitlines(),
            Equals(
                [
                    "Git (`git`) is not available, this tool cannot verify its "
                    "prerequisites.",
                    "Please install it before trying this command again:",
                    "",
                    "    $ sudo apt install git",
                ]
            ),
        )

    @mock.patch("subprocess.check_call")
    def test_broken_git_repo(self, mock_check_call):
        mock_check_call.side_effect = [None, subprocess.CalledProcessError(1, "test")]

        raised = self.assertRaises(
            RequiredCommandFailure, self.run_command, self.entry_point
        )

        self.assertThat(
            str(raised).splitlines(),
            Equals(
                [
                    "The current directory is not a Git repository.",
                    "Please switch to the desired project repository where "
                    "Travis should be enabled.",
                ]
            ),
        )

    @mock.patch("subprocess.check_call")
    def test_missing_travis_setup(self, mock_check_call):
        mock_check_call.side_effect = [None, None]

        raised = self.assertRaises(
            RequiredPathDoesNotExist, self.run_command, self.entry_point
        )

        self.assertThat(
            str(raised).splitlines(),
            Equals(
                [
                    "Travis project is not initialized for the current directory.",
                    "Please initialize Travis project (e.g. `travis init`) with "
                    "appropriate parameters.",
                ]
            ),
        )


class TravisSuccessfulTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)

    def make_travis_yml(self, content):
        with open(".travis.yml", "w") as fd:
            fd.write(content)

    @mock.patch("subprocess.check_output")
    @mock.patch("subprocess.check_call")
    @mock.patch("builtins.input")
    @mock.patch("getpass.getpass")
    @mock.patch.object(storeapi.StoreClient, "login")
    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    def test_enable_successfully(
        self,
        mock_get_account_information,
        mock_login,
        mock_getpass,
        mock_input,
        mock_check_call,
        mock_check_output,
    ):
        mock_input.side_effect = ["sample.person@canonical.com", "123456"]
        mock_getpass.return_value = "secret"
        mock_login.side_effect = [
            storeapi.errors.StoreTwoFactorAuthenticationRequired(),
            None,
        ]
        mock_get_account_information.return_value = {"account_id": "abcd"}

        mock_check_call.side_effect = [None, None]
        mock_check_output.side_effect = [None]
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(test_snapcraft_yaml)
        project = Project(snapcraft_yaml_file_path=snapcraft_yaml_file_path)
        self.make_travis_yml('after_success: ["<travis-cli-decrypt>"]')

        travis.enable(project)

        # Attenuated credentials requested from the Store.
        mock_login.assert_called_with(
            "sample.person@canonical.com",
            "secret",
            one_time_password="123456",
            save=False,
            acls=["package_access", "package_push", "package_release"],
            channels=["edge"],
            packages=[{"series": "16", "name": "foo"}],
            expires=None,
            config_fd=None,
        )

        # Credentials encrypted with travis CLI.
        mock_check_output.assert_called_with(
            [
                "travis",
                "encrypt-file",
                "--force",
                "--add",
                "after_success",
                "--decrypt-to",
                travis.LOCAL_CONFIG_FILENAME,
                mock.ANY,
                travis.ENCRYPTED_CONFIG_FILENAME,
            ],
            stderr=subprocess.PIPE,
        )

        # '.travis.yml' updated for snap CI.
        with open(".travis.yml") as fd:
            travis_conf = yaml.load(fd)
            self.assertThat(travis_conf["sudo"], Equals("required"))
            self.assertThat(travis_conf["services"], Equals(["docker"]))
            self.assertThat(
                travis_conf["after_success"], Equals(["<travis-cli-decrypt>"])
            )
            self.assertThat(
                travis_conf["deploy"],
                Equals(
                    {
                        "skip_cleanup": True,
                        "provider": "script",
                        "script": (
                            "docker run -v $(pwd):$(pwd) -t snapcore/snapcraft "
                            'sh -c "apt update -qq && '
                            "cd $(pwd) && "
                            'snapcraft && snapcraft push *.snap --release edge"'
                        ),
                        "on": {"branch": "master"},
                    }
                ),
            )

        # Descriptive logging ...
        self.assertThat(
            self.fake_logger.output,
            Contains(
                dedent(
                    """\
                Enabling Travis testbeds to push and release 'foo' snaps to edge channel in series '16'
                Acquiring specific authorization information ...
                Encrypting authorization for Travis and adjusting project to automatically decrypt and use it during "after_success".
                Configuring "deploy" phase to build and release the snap in the Store.
                Done. Now you just have to review and commit changes in your Travis project (`.travis.yml`).
                Also make sure you add the new `.snapcraft/travis_snapcraft.cfg` file.
            """
                )
            ),
        )  # noqa TODO this type of test should not be done

    @mock.patch("subprocess.check_output")
    @mock.patch("subprocess.check_call")
    @mock.patch("builtins.input")
    @mock.patch("getpass.getpass")
    @mock.patch.object(storeapi.StoreClient, "login")
    @mock.patch.object(storeapi._sca_client.SCAClient, "get_account_information")
    def test_refresh_successfully(
        self,
        mock_get_account_information,
        mock_login,
        mock_getpass,
        mock_input,
        mock_check_call,
        mock_check_output,
    ):
        mock_input.side_effect = ["sample.person@canonical.com", "123456"]
        mock_getpass.return_value = "secret"
        mock_login.side_effect = [
            storeapi.errors.StoreTwoFactorAuthenticationRequired(),
            None,
        ]
        mock_get_account_information.return_value = {"account_id": "abcd"}

        mock_check_call.side_effect = [None, None]
        mock_check_output.side_effect = [None]
        snapcraft_yaml_file_path = self.make_snapcraft_yaml(test_snapcraft_yaml)
        project = Project(snapcraft_yaml_file_path=snapcraft_yaml_file_path)
        self.make_travis_yml('after_success: ["<travis-cli-decrypt>"]')

        travis.refresh(project)

        # Attenuated credentials requested from the Store.
        mock_login.assert_called_with(
            "sample.person@canonical.com",
            "secret",
            one_time_password="123456",
            save=False,
            acls=["package_access", "package_push", "package_release"],
            channels=["edge"],
            packages=[{"series": "16", "name": "foo"}],
            expires=None,
            config_fd=None,
        )

        # Credentials encrypted with travis CLI.
        mock_check_output.assert_called_with(
            [
                "travis",
                "encrypt-file",
                "--force",
                "--add",
                "after_success",
                "--decrypt-to",
                travis.LOCAL_CONFIG_FILENAME,
                mock.ANY,
                travis.ENCRYPTED_CONFIG_FILENAME,
            ],
            stderr=subprocess.PIPE,
        )

        # '.travis.yml' updated only with the decrypt command.
        with open(".travis.yml") as fd:
            travis_conf = yaml.load(fd)
            self.assertThat(
                travis_conf["after_success"], Equals(["<travis-cli-decrypt>"])
            )

        # Descriptive logging ...
        self.assertThat(
            self.fake_logger.output,
            Contains(
                dedent(
                    """\
                Refreshing credentials to push and release "foo" snaps to edge channel in series 16
                Acquiring specific authorization information ...
                Encrypting authorization for Travis and adjusting project to automatically decrypt and use it during "after_success".
                Done. Please commit the changes to `.snapcraft/travis_snapcraft.cfg` file.
            """
                )
            ),
        )  # noqa TODO this type of test should not be done
