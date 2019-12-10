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

import os
from collections import OrderedDict
from pathlib import Path
from textwrap import dedent

import fixtures
from testtools.matchers import Equals

from snapcraft import yaml_utils
from snapcraft.internal.meta import errors
from snapcraft.internal.meta.snap import SystemUserScope
from snapcraft.internal.meta.snap import Snap
from tests import unit


class SnapYamlTests(unit.TestCase):
    # The YAMLs must be formatted and ordered as output
    # from Snap.to_snap_yaml() to simplify checking and re-use
    # the source YAML as the expected output YAML.
    scenarios = [
        (
            "minimal",
            dict(
                yaml=dedent(
                    """\
                name: test-name
                version: v1
                summary: test-summary
                description: test-description
            """
                )
            ),
        ),
        (
            "multiple-arches",
            dict(
                yaml=dedent(
                    """\
                name: test-name
                version: '1.0'
                summary: test-summary
                description: test-description
                architectures:
                - i386
                - amd64
                """
                )
            ),
        ),
        (
            "system-usernames-longform",
            dict(
                yaml=dedent(
                    """\
                name: test-name
                version: '1.0'
                summary: test-summary
                description: test-description
                system-usernames:
                  snap_daemon:
                    scope: shared
                """
                )
            ),
        ),
        (
            "system-usernames-multiple-longform",
            dict(
                yaml=dedent(
                    """\
                name: test-name
                version: '1.0'
                summary: test-summary
                description: test-description
                system-usernames:
                  lxd:
                    scope: shared
                  snap_daemon:
                    scope: shared
                """
                )
            ),
        ),
        (
            "full",
            dict(
                yaml=dedent(
                    """\
                name: test
                version: v1
                summary: test
                description: test
                apps:
                  app1:
                    command: app1
                    extensions:
                    - environment
                  app2:
                    command: app2
                    extensions:
                    - environment
                architectures:
                - all
                assumes:
                - command-chain
                base: core18
                confinement: strict
                environment:
                  TMPDIR: $XDG_RUNTIME_DIR
                epoch: 0
                grade: devel
                hooks:
                  test-hook:
                    plugs:
                    - network
                layout:
                  /target:
                    bind: $SNAP/foo
                license: GPL
                plugs:
                  test-plug:
                    interface: nil
                  test-plug2:
                    interface: nil
                slots:
                  test-slot:
                    interface: nil
                  test-slot2:
                    interface: nil
                title: sometitle
                type: base
                """
                )
            ),
        ),
    ]

    def test_scenarios(self):
        snap_yaml_path = Path("snap.yaml")
        open(snap_yaml_path, "w").write(self.yaml)
        snap_yaml_dict = yaml_utils.load_yaml_file(snap_yaml_path)

        # Create validated Snap object from snap.yaml.
        snap = Snap.from_snap_yaml(snap_yaml_path)
        snap.validate()

        # Write snap.yaml from Snap object.
        written_yaml_path = Path("out-snap.yaml")
        snap.write_snap_yaml(written_yaml_path)

        # Read written snap.yaml.
        read_yaml = open(written_yaml_path).read()
        read_yaml_dict = yaml_utils.load_yaml_file(written_yaml_path)

        # The YAMLs objects should match.
        self.assertThat(snap_yaml_dict, Equals(read_yaml_dict))

        # The YAML files should match if formatted correctly above.
        self.assertThat(read_yaml, Equals(self.yaml))


class SnapcraftYamlTests(unit.TestCase):
    # The YAMLs must be formatted and ordered as output
    # from Snap.to_snapcraft_yaml() to simplify checking.
    scenarios = [
        (
            "minimal",
            dict(
                yaml=dedent(
                    """\
                name: test-name
                version: v1
                summary: test-summary
                description: test-description
            """
                )
            ),
        ),
        (
            "system-usernames-longform",
            dict(
                yaml=dedent(
                    """\
                name: test-name
                version: v1
                summary: test-summary
                description: test-description
                system-usernames:
                  snap_daemon:
                    scope: shared
                """
                )
            ),
        ),
        (
            "system-usernames-multiple-longform",
            dict(
                yaml=dedent(
                    """\
                name: test-name
                version: v1
                summary: test-summary
                description: test-description
                system-usernames:
                  lxd:
                    scope: shared
                  snap_daemon:
                    scope: shared
                """
                )
            ),
        ),
        (
            "full",
            dict(
                yaml=dedent(
                    """\
                name: test
                version: v1
                summary: test
                description: test
                apps:
                  app1:
                    command: app1
                    extensions:
                    - environment
                  app2:
                    command: app2
                    extensions:
                    - environment
                architectures:
                - all
                assumes:
                - command-chain
                base: core
                confinement: strict
                environment:
                  TMPDIR: $XDG_RUNTIME_DIR
                epoch: 0
                grade: devel
                hooks:
                  test-hook:
                    plugs:
                    - network
                layout:
                  /target:
                    bind: $SNAP/foo
                license: GPL
                passthrough:
                  somekey: somevalue
                plugs:
                  test-plug:
                    interface: nil
                  test-plug2:
                    interface: nilgit dif
                  test-slot:
                    interface: nil
                  test-slot2:
                    interface: nil
                title: sometitle
                type: base
                """
                )
            ),
        ),
    ]

    def test_scenarios(self):
        snapcraft_yaml_path = Path("snapcraft.yaml")
        open(snapcraft_yaml_path, "w").write(self.yaml)
        snapcraft_yaml_dict = yaml_utils.load_yaml_file(snapcraft_yaml_path)

        # Create validated Snap object from snapcraft.yaml.
        snap = Snap.from_snapcraft_yaml(snapcraft_yaml_path)
        snap.validate()

        # Write snapcraft.yaml from Snap object.
        written_yaml_path = Path("out-snapcraft.yaml")
        snap.write_snapcraft_yaml(written_yaml_path)

        # Read written snapcraft.yaml.
        read_yaml = open(written_yaml_path).read()
        read_yaml_dict = yaml_utils.load_yaml_file(written_yaml_path)

        # The YAMLs objects should match.
        self.assertThat(snapcraft_yaml_dict, Equals(read_yaml_dict))

        # The YAML files should match if formatted correctly above.
        self.assertThat(read_yaml, Equals(self.yaml))


class SnapTests(unit.TestCase):
    def test_simple(self):
        snap = Snap(
            name="snap-test",
            base="fake-base",
            version="snap-version",
            summary="snap-summary",
            description="snap-description",
            grade="stable",
        )

        snap.validate()
        self.assertEqual(False, snap.is_passthrough_enabled)
        self.assertEqual("snap-test", snap.name)
        self.assertEqual("snap-version", snap.version)
        self.assertEqual("snap-summary", snap.summary)
        self.assertEqual("snap-description", snap.description)
        self.assertEqual("stable", snap.grade)

    def test_missing_keys(self):
        snap = Snap(name="snap-test", grade="stable")

        self.assertRaises(errors.MissingSnapcraftYamlKeysError, snap.validate)

    def test_snap_yaml_passthrough(self):
        passthrough = {"passthrough-key": "passthrough-value"}
        snap = Snap(
            name="snap-test",
            base="fake-base",
            version="snap-version",
            summary="snap-summary",
            description="snap-description",
            grade="stable",
            passthrough=passthrough,
        )

        snap.validate()

        transformed_dict = snap.to_snap_yaml_dict()

        self.assertTrue(snap.is_passthrough_enabled)
        self.assertThat(snap.passthrough, Equals(passthrough))
        self.assertThat(
            transformed_dict["passthrough-key"], Equals("passthrough-value")
        )
        self.assertFalse("passthrough" in transformed_dict)

    def test_all_keys_snapcraft_yaml_dict(self):
        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "version": "test-version",
                "summary": "test-summary",
                "description": "test-description",
                "apps": {"test-app": {"command": "test-app"}},
                "architectures": ["all"],
                "assumes": ["command-chain"],
                "base": "core18",
                "confinement": "strict",
                "environment": {"TESTING": "1"},
                "epoch": 0,
                "grade": "devel",
                "hooks": {"test-hook": {"plugs": ["network"]}},
                "layout": {"/target": {"bind": "$SNAP/foo"}},
                "license": "GPL",
                "plugs": {"test-plug": OrderedDict({"interface": "some-value"})},
                "slots": {"test-slot": OrderedDict({"interface": "some-value"})},
                "system-usernames": {"snap_daemon": {"scope": "shared"}},
                "title": "test-title",
                "type": "base",
            }
        )

        snap = Snap.from_snapcraft_yaml_dict(snap_dict=snap_dict)
        snap.validate()

        self.assertEqual(snap_dict, snap.to_snap_yaml_dict())
        self.assertFalse(snap.is_passthrough_enabled)
        self.assertEqual(snap_dict["name"], snap.name)
        self.assertEqual(snap_dict["version"], snap.version)
        self.assertEqual(snap_dict["summary"], snap.summary)
        self.assertEqual(snap_dict["description"], snap.description)
        self.assertEqual(snap_dict["apps"]["test-app"], snap.apps["test-app"].to_dict())
        self.assertEqual(snap_dict["architectures"], snap.architectures)
        self.assertEqual(set(snap_dict["assumes"]), snap.assumes)
        self.assertEqual(snap_dict["base"], snap.base)
        self.assertEqual(snap_dict["environment"], snap.environment)
        self.assertEqual(snap_dict["license"], snap.license)
        self.assertEqual(
            snap_dict["plugs"]["test-plug"], snap.plugs["test-plug"].to_dict()
        )
        self.assertEqual(
            snap_dict["slots"]["test-slot"], snap.slots["test-slot"].to_dict()
        )
        self.assertEqual(snap_dict["confinement"], snap.confinement)
        self.assertEqual(snap_dict["title"], snap.title)
        self.assertEqual(snap_dict["type"], snap.type)

    def test_system_usernames_shortform_scope(self):
        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "version": "test-version",
                "summary": "test-summary",
                "description": "test-description",
                "system-usernames": {"snap_daemon": "shared", "lxd": "shared"},
            }
        )

        snap = Snap.from_snapcraft_yaml_dict(snap_dict=snap_dict)
        snap.validate()

        self.assertThat(
            snap.system_usernames["snap_daemon"].name, Equals("snap_daemon")
        )
        self.assertThat(
            snap.system_usernames["snap_daemon"].scope, Equals(SystemUserScope.SHARED)
        )
        self.assertThat(snap.system_usernames["lxd"].name, Equals("lxd"))
        self.assertThat(
            snap.system_usernames["lxd"].scope, Equals(SystemUserScope.SHARED)
        )

    def test_system_usernames_longform_scope(self):
        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "version": "test-version",
                "summary": "test-summary",
                "description": "test-description",
                "system-usernames": {
                    "snap_daemon": {"scope": "shared"},
                    "lxd": {"scope": "shared"},
                },
            }
        )

        snap = Snap.from_snapcraft_yaml_dict(snap_dict=snap_dict)
        snap.validate()

        self.assertThat(
            snap.system_usernames["snap_daemon"].name, Equals("snap_daemon")
        )
        self.assertThat(
            snap.system_usernames["snap_daemon"].scope, Equals(SystemUserScope.SHARED)
        )
        self.assertThat(snap.system_usernames["lxd"].name, Equals("lxd"))
        self.assertThat(
            snap.system_usernames["lxd"].scope, Equals(SystemUserScope.SHARED)
        )

    def test_is_passthrough_enabled_app(self):
        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "version": "test-version",
                "summary": "test-summary",
                "description": "test-description",
                "apps": {
                    "test-app": {
                        "command": "test-app",
                        "passthrough": {"some-key": "some-value"},
                    }
                },
            }
        )

        snap = Snap.from_snapcraft_yaml_dict(snap_dict=snap_dict)
        snap.validate()

        self.assertTrue(snap.is_passthrough_enabled)

    def test_is_passthrough_enabled_hook(self):
        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "version": "test-version",
                "summary": "test-summary",
                "description": "test-description",
                "hooks": {"test-hook": {"passthrough": {"some-key": "some-value"}}},
            }
        )

        snap = Snap.from_snapcraft_yaml_dict(snap_dict=snap_dict)
        snap.validate()

        self.assertTrue(snap.is_passthrough_enabled)

    def test_get_provider_content_directories_no_plugs(self):
        snap = Snap()
        self.assertEqual(set([]), snap.get_provider_content_directories())

    def test_get_provider_content_directories_with_content_plugs(self):
        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "version": "test-version",
                "summary": "test-summary",
                "description": "test-description",
                "plugs": {
                    "test-plug": {
                        "interface": "content",
                        "content": "content",
                        "target": "target",
                        "default-provider": "gtk-common-themes:gtk-3-themes",
                    }
                },
            }
        )

        meta_snap_yaml = dedent(
            """
            name: test-content-snap-meta-snap-yaml
            version: "1.0"
            summary: test-summary
            description: test-description
            base: core18
            architectures:
            - all
            confinement: strict
            grade: stable
            slots:
              test-slot-name:
                interface: content
                source:
                  read:
                  - $SNAP/dir1
                  - $SNAP/dir2
        """
        )

        snap = Snap.from_snapcraft_yaml_dict(snap_dict=snap_dict)
        snap.validate()

        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.common.get_installed_snap_path",
                return_value=self.path,
            )
        )

        meta_path = Path(self.path, "meta")
        os.makedirs(meta_path)

        snap_yaml_path = Path(meta_path, "snap.yaml")
        open(snap_yaml_path, "w").write(meta_snap_yaml)

        expected_content_dirs = set(
            [os.path.join(self.path, "dir1"), os.path.join(self.path, "dir2")]
        )

        self.assertEqual(expected_content_dirs, snap.get_provider_content_directories())

    def test_ensure_command_chain_assumption(self):
        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "version": "snap-version",
                "summary": "snap-summary",
                "description": "snap-description",
                "apps": {
                    "test-app": {
                        "command": "test-app",
                        "command-chain": ["test-command-chain"],
                    }
                },
            }
        )

        snap = Snap.from_snapcraft_yaml_dict(snap_dict=snap_dict)
        snap._ensure_command_chain_assumption()
        snap.validate()

        self.assertEqual({"command-chain"}, snap.assumes)

    def test_write_snap_yaml_skips_base_core(self):
        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "version": "snap-version",
                "summary": "snap-summary",
                "description": "snap-description",
                "base": "core",
                "grade": "devel",
            }
        )

        snap = Snap.from_snapcraft_yaml_dict(snap_dict=snap_dict)
        snap.validate()

        # Write snap yaml.
        snap_yaml_path = Path(self.path, "snap.yaml")
        snap.write_snap_yaml(snap_yaml_path)

        # Read snap yaml.
        written_snap_yaml = open(snap_yaml_path, "r").read()

        self.assertFalse("base" in written_snap_yaml)
