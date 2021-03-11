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

import os

import xdg
from testtools.matchers import Equals

from snapcraft import config
from snapcraft.internal.errors import SnapcraftInvalidCLIConfigError
from tests import unit


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
