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

from testtools.matchers import Equals, FileContains, Is

from snapcraft.internal.meta._snap_packaging import _SnapPackaging
from snapcraft.internal.project_loader import load_config
from snapcraft.project import Project
from tests import fixture_setup, unit


class SnapPackagingRunnerTests(unit.TestCase):
    def _get_snap_packaging(self, **yaml_args):
        if "parts" not in yaml_args:
            yaml_args["parts"] = dict(part1=dict(plugin="nil"))

        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path, **yaml_args)
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
            export PATH="$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin${PATH:+:$PATH}"
            export LD_LIBRARY_PATH="$SNAP_LIBRARY_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            exec "$@"
            """
        ).lstrip()

        self.assertThat(snapcraft_runner, Equals(expected_runner))

    def test_classic_app_with_snapd_workaround(self):
        """Test workaround for classic (LP: #1860369)."""
        apps = dict(testapp=dict(command="echo"))

        for base in ("core", "core16", "core18"):
            sp = self._get_snap_packaging(apps=apps, base=base, confinement="classic")
            runner = sp._generate_snapcraft_runner()

            self.expectThat(runner, Equals("snap/command-chain/snapcraft-runner"))
            runner_path = Path(self.path, "prime", runner)
            self.expectThat(
                runner_path,
                FileContains(
                    textwrap.dedent(
                        """
            #!/bin/sh

            exec "$@"
            """
                    ).lstrip()
                ),
            )

    def test_classic_app_without_snapd_workaround(self):
        """Test no workaround for classic (LP: #1860369)."""
        apps = dict(testapp=dict(command="echo"))

        sp = self._get_snap_packaging(apps=apps, base="core20", confinement="classic")
        runner = sp._generate_snapcraft_runner()

        self.expectThat(runner, Is(None))

    def test_snapd(self):
        apps = dict(testapp=dict(command="echo"))

        sp = self._get_snap_packaging(
            apps=apps,
            confinement="classic",
            type="snapd",
            base=None,
            build_base="core18",
        )
        runner = sp._generate_snapcraft_runner()

        self.assertThat(runner, Equals(None))

    def test_assembled_runtime_environment_classic(self):
        apps = dict(testapp=dict(command="echo"))

        sp = self._get_snap_packaging(
            apps=apps, confinement="classic", type="app", base="core"
        )

        assembled_env = sp._assemble_runtime_environment()

        self.assertThat(assembled_env, Equals(""))

    def test_assembled_runtime_environment_strict(self):
        apps = dict(testapp=dict(command="echo"))

        sp = self._get_snap_packaging(
            apps=apps, confinement="strict", type="app", base="core"
        )

        assembled_env = sp._assemble_runtime_environment()

        expected_env = textwrap.dedent(
            """
            export PATH="$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin${PATH:+:$PATH}"
            export LD_LIBRARY_PATH="$SNAP_LIBRARY_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            """
        ).strip()

        self.assertThat(assembled_env, Equals(expected_env))

    def test_assembled_runtime_environment_strict_patched(self):
        apps = dict(testapp=dict(command="echo"))
        parts = {"part1": {"plugin": "nil", "build-attributes": ["enable-patchelf"]}}

        sp = self._get_snap_packaging(
            apps=apps, parts=parts, confinement="strict", type="app", base="core"
        )

        assembled_env = sp._assemble_runtime_environment()

        # Verify that, since all parts are using patchelf, no LD_LIBRARY_PATH is set
        expected_env = textwrap.dedent(
            """
            export PATH="$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin${PATH:+:$PATH}"
            """
        ).strip()

        self.assertThat(assembled_env, Equals(expected_env))

    def test_stub_hook(self):
        apps = dict(testapp=dict(command="echo"))
        parts = {"part1": {"plugin": "nil", "build-attributes": ["enable-patchelf"]}}
        hooks = dict(configure={"command-chain": ["foo"], "plugs": ["network"]})

        sp = self._get_snap_packaging(
            apps=apps,
            parts=parts,
            confinement="strict",
            type="app",
            base="core",
            hooks=hooks,
        )

        sp.generate_hook_wrappers()

        hook = Path(sp._prime_dir, "meta", "hooks", "configure")
        assert hook.exists()
        assert hook.read_text() == "#!/bin/sh\n"
        assert hook.stat().st_mode & 0xFFF == 0o755

    def test_no_stub_for_preconfigured_hook(self):
        apps = dict(testapp=dict(command="echo"))
        parts = {"part1": {"plugin": "nil", "build-attributes": ["enable-patchelf"]}}
        hooks = dict(configure={"command-chain": ["foo"], "plugs": ["network"]})

        sp = self._get_snap_packaging(
            apps=apps,
            parts=parts,
            confinement="strict",
            type="app",
            base="core",
            hooks=hooks,
        )

        hook = Path(sp._prime_dir, "meta", "hooks", "configure")
        hook.parent.mkdir(parents=True)
        hook.write_text("foo")

        sp.generate_hook_wrappers()

        assert hook.exists()
        assert hook.read_text() == "foo"
