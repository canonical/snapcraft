# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import textwrap
from pathlib import Path

from testtools.matchers import Equals

from snapcraft.internal.meta._snap_packaging import _SnapPackaging
from snapcraft.internal.project_loader import load_config
from snapcraft.project import Project
from tests import fixture_setup, unit


class SnapPackagingRunnerTests(unit.TestCase):
    def _get_snap_packaging(self, **yaml_args):
        parts = dict(part1=dict(plugin="nil"))

        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path, parts=parts, **yaml_args
        )
        self.useFixture(snapcraft_yaml)

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        config = load_config(project)

        return _SnapPackaging(project_config=config, extracted_metadata=None)

    def test_no_apps(self):
        apps = dict()

        sp = self._get_snap_packaging(apps=apps, confinement="strict")
        runner = sp._generate_snapcraft_runner()

        self.assertThat(runner, Equals(None))

        sp = self._get_snap_packaging(apps=apps, confinement="strict")
        runner = sp._generate_snapcraft_runner()

        self.assertThat(runner, Equals(None))

    def test_strict_app(self):
        apps = dict(testapp=dict(command="echo"))

        sp = self._get_snap_packaging(apps=apps, confinement="strict")
        runner = sp._generate_snapcraft_runner()

        self.assertThat(runner, Equals("snap/command-chain/snapcraft-runner"))

        runner_path = Path(self.path, "prime", runner)

        with open(runner_path, "r") as f:
            snapcraft_runner = f.read()

        expected_runner = textwrap.dedent(
            """
            #!/bin/sh
            export PATH="$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH"
            export LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH
            exec "$@"
            """
        ).lstrip()

        self.assertThat(snapcraft_runner, Equals(expected_runner))

    def test_classic_app(self):
        apps = dict(testapp=dict(command="echo"))

        sp = self._get_snap_packaging(apps=apps, confinement="classic")
        runner = sp._generate_snapcraft_runner()

        self.assertThat(runner, Equals("snap/command-chain/snapcraft-runner"))

        runner_path = Path(self.path, "prime", runner)

        with open(runner_path, "r") as f:
            snapcraft_runner = f.read()

        expected_runner = textwrap.dedent(
            """
            #!/bin/sh

            exec "$@"
            """
        ).lstrip()

        self.assertThat(snapcraft_runner, Equals(expected_runner))

    def test_snapd(self):
        apps = dict(testapp=dict(command="echo"))

        sp = self._get_snap_packaging(
            apps=apps, confinement="classic", type="snapd", base=None
        )
        runner = sp._generate_snapcraft_runner()

        self.assertThat(runner, Equals(None))
