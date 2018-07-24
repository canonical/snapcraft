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

import fixtures
import os
import xdg

from testtools.matchers import Contains, Equals, FileContains

from snapcraft import config
from snapcraft.internal.errors import SnapcraftInvalidCLIConfigError
from snapcraft.storeapi import errors
from tests import unit


def create_config_from_string(content):
    path = config.Config.save_path()
    with open(path, "w") as f:
        f.write(content)


class TestCLIConfig(unit.TestCase):
    def test_non_existing_file_succeeds(self):
        conf = config.CLIConfig()
        self.assertThat(conf.parser.sections(), Equals([]))

    def test_set_and_get_sentry_send_always_with_contextmanager(self):
        with config.CLIConfig() as cli_config:
            cli_config.set_sentry_send_always(True)

        with config.CLIConfig() as cli_config:
            self.assertThat(cli_config.get_sentry_send_always(), Equals(True))

    def test_set_and_get_sentry_send_always(self):
        cli_config = config.CLIConfig()
        cli_config.set_sentry_send_always(True)
        cli_config.save()

        new_cli_config = config.CLIConfig()
        new_cli_config.load()
        self.assertThat(new_cli_config.get_sentry_send_always(), Equals(True))

    def test_set_and_get_outdated_step_action_with_contextmanager(self):
        with config.CLIConfig() as cli_config:
            cli_config.set_outdated_step_action(config.OutdatedStepAction.CLEAN)

        with config.CLIConfig() as cli_config:
            self.assertThat(
                cli_config.get_outdated_step_action(),
                Equals(config.OutdatedStepAction.CLEAN),
            )

    def test_set_and_get_outdated_step_action(self):
        cli_config = config.CLIConfig()
        cli_config.set_outdated_step_action(config.OutdatedStepAction.CLEAN)
        cli_config.save()

        new_cli_config = config.CLIConfig()
        new_cli_config.load()
        self.assertThat(
            new_cli_config.get_outdated_step_action(),
            Equals(config.OutdatedStepAction.CLEAN),
        )

    def test_set_when_read_only(self):
        cli_config = config.CLIConfig(read_only=True)

        self.assertRaises(RuntimeError, cli_config.set_sentry_send_always, True)

    def test_save_when_read_only(self):
        cli_config = config.CLIConfig(read_only=True)

        self.assertRaises(RuntimeError, cli_config.save)

    def test_contextmanager_with_read_only(self):
        with config.CLIConfig(read_only=True) as cli_config:
            # This should be False
            self.assertThat(cli_config.get_sentry_send_always(), Equals(False))

    def test_load_invalid_config(self):
        config_path = os.path.join(
            xdg.BaseDirectory.save_config_path("snapcraft"), "cli.cfg"
        )
        with open(config_path, "w") as f:
            f.write("invalid config")

        cli_config = config.CLIConfig()

        self.assertRaises(SnapcraftInvalidCLIConfigError, cli_config.load)


class TestConfig(unit.TestCase):
    def test_non_existing_file_succeeds(self):
        conf = config.Config()
        self.assertThat(conf.parser.sections(), Equals([]))
        self.assertTrue(conf.is_empty())

    def test_existing_file(self):
        existing_conf = config.Config()
        existing_conf.set("foo", "bar")
        existing_conf.save()
        # Check we find and use the existing conf
        conf = config.Config()
        self.assertThat(conf.get("foo"), Equals("bar"))
        self.assertFalse(conf.is_empty())

    def test_irrelevant_sections_are_ignored(self):
        create_config_from_string("""[example.com]\nfoo=bar""")
        conf = config.Config()
        self.assertThat(conf.get("foo"), Equals(None))

    def test_section_from_url(self):
        create_config_from_string("""[example.com]\nfoo=bar""")
        self.useFixture(
            fixtures.EnvironmentVariable(
                "UBUNTU_SSO_API_ROOT_URL", "http://example.com/api/v2"
            )
        )
        conf = config.Config()
        self.assertThat(conf.get("foo"), Equals("bar"))

    def test_save_one_option(self):
        conf = config.Config()
        conf.set("bar", "baz")
        conf.save()
        new_conf = config.Config()
        self.assertThat(new_conf.get("bar"), Equals("baz"))

    def test_clear_preserver_other_sections(self):
        create_config_from_string("""[keep_me]\nfoo=bar\n""")
        conf = config.Config()
        conf.set("bar", "baz")
        self.assertThat(conf.get("bar"), Equals("baz"))
        conf.clear()
        conf.save()
        new_conf = config.Config()
        self.assertThat(new_conf.get("bar"), Equals(None))
        # Picking behind the curtains
        self.assertThat(new_conf.parser.get("keep_me", "foo"), Equals("bar"))
        self.assertTrue(conf.is_empty())

    def test_save_encoded(self):
        conf = config.Config()
        conf.set("bar", "baz")
        conf.save(encode=True)
        new_conf = config.Config()
        self.assertThat(new_conf.get("bar"), Equals("baz"))

    def test_save_encoded_to_file(self):
        conf = config.Config()
        conf.set("bar", "baz")
        with open("test-config", "w") as f:
            conf.save(config_fd=f, encode=True)
            f.flush()

        self.assertThat(
            "test-config", FileContains("W2xvZ2luLnVidW50dS5jb21dCmJhciA9IGJhegoK")
        )

        new_conf = config.Config()
        with open("test-config", "r") as f:
            new_conf.load(config_fd=f)
        self.assertThat(new_conf.get("bar"), Equals("baz"))

    def test_load_invalid_config(self):
        with open("test-config", "w") as f:
            f.write("invalid config")
            f.flush()

        conf = config.Config()
        with open("test-config", "r") as f:
            raised = self.assertRaises(
                errors.InvalidLoginConfig, conf.load, config_fd=f
            )

        self.assertThat(str(raised), Contains("File contains no section headers"))


class TestOptions(unit.TestCase):
    def create_config(self, **kwargs):
        conf = config.Config()
        for k, v in kwargs.items():
            conf.set(k, v)
        return conf

    def test_string(self):
        conf = self.create_config(foo="bar")
        self.assertThat(conf.get("foo"), Equals("bar"))


def create_local_config_from_string(content):
    os.makedirs(os.path.dirname(config.LOCAL_CONFIG_FILENAME), exist_ok=True)
    with open(config.LOCAL_CONFIG_FILENAME, "w") as f:
        f.write(content)


class TestLocalConfig(unit.TestCase):
    def setUp(self):
        super().setUp()
        override_sso = fixtures.EnvironmentVariable(
            "UBUNTU_SSO_API_ROOT_URL", "http://example.com/api/v2"
        )
        self.useFixture(override_sso)

    def test_local_config_is_considered(self):
        create_local_config_from_string("""[example.com]\nfoo=bar""")
        conf = config.Config()
        self.assertThat(conf.get("foo"), Equals("bar"))

    def test_local_config_is_preferred(self):
        create_config_from_string("""[example.com]\nfoo=baz""")
        create_local_config_from_string("""[example.com]\nfoo=bar""")
        conf = config.Config()
        self.assertThat(conf.get("foo"), Equals("bar"))

    def test_local_config_is_static(self):
        create_local_config_from_string("""[example.com]\nfoo=bar""")
        conf = config.Config()
        conf.set("foo", "baz")
        conf.save()
        new_conf = config.Config()
        self.assertThat(new_conf.get("foo"), Equals("bar"))
