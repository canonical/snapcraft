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
from textwrap import dedent
from unittest import mock

import testscenarios
from testtools.matchers import Equals

from snapcraft.internal.meta import errors
from snapcraft.internal.meta.system_user import SystemUserScope
from snapcraft.internal.meta.snap import Snap
from tests import integration, unit


class SnapTests(unit.TestCase):
    """ Test the snaps.  Note that the ordering of ordereddicts must align
    with Snap's use of _MANDATORY_PACKAGE_KEYS + _OPTIONAL_PACKAGE_KEYS.

    This applies even for verifying YAMLs, which are (now) ordered."""

    def test_empty(self):
        snap_dict = OrderedDict()

        snap = Snap()

        self.assertEqual(snap_dict, snap.to_dict())

    def test_empty_from_dict(self):
        snap_dict = OrderedDict({})

        snap = Snap.from_dict(snap_dict=snap_dict)

        self.assertEqual(snap_dict, snap.to_dict())

    def test_missing_keys(self):
        snap_dict = OrderedDict({"name": "snap-test", "grade": "stable"})

        snap = Snap.from_dict(snap_dict=snap_dict)

        self.assertEqual(snap_dict, snap.to_dict())
        self.assertRaises(errors.MissingSnapcraftYamlKeysError, snap.validate)

    def test_grade_devel_statisfies_required_grade(self):
        self.fake_snapd.snaps_result = [
            {"name": "fake-base", "channel": "edge", "revision": "fake-revision"}
        ]

        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "base": "fake-base",
                "version": "snap-version",
                "summary": "snap-summary",
                "description": "snap-description",
                "grade": "devel",
            }
        )

        snap = Snap.from_dict(snap_dict=snap_dict)

        snap.validate()

    def test_simple(self):
        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "version": "snap-version",
                "summary": "snap-summary",
                "description": "snap-description",
                "grade": "stable",
            }
        )

        snap = Snap.from_dict(snap_dict=snap_dict)
        snap.validate()

        self.assertEqual(snap_dict, snap.to_dict())
        self.assertEqual(False, snap.is_passthrough_enabled)
        self.assertEqual(snap_dict["name"], snap.name)
        self.assertEqual(snap_dict["version"], snap.version)
        self.assertEqual(snap_dict["summary"], snap.summary)
        self.assertEqual(snap_dict["description"], snap.description)

    def test_passthrough(self):
        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "version": "snap-version",
                "summary": "snap-summary",
                "description": "snap-description",
                "passthrough": {"otherkey": "othervalue"},
                "grade": "stable",
            }
        )

        snap = Snap.from_dict(snap_dict=snap_dict)
        snap.validate()

        transformed_dict = snap_dict.copy()
        passthrough = transformed_dict.pop("passthrough")
        transformed_dict.update(passthrough)

        self.assertEqual(transformed_dict, snap.to_dict())
        self.assertEqual(True, snap.is_passthrough_enabled)
        self.assertEqual(passthrough, snap.passthrough)
        self.assertEqual(snap_dict["name"], snap.name)
        self.assertEqual(snap_dict["version"], snap.version)
        self.assertEqual(snap_dict["summary"], snap.summary)
        self.assertEqual(snap_dict["description"], snap.description)

    def test_all_keys(self):
        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "version": "test-version",
                "summary": "test-summary",
                "description": "test-description",
                "apps": {"test-app": {"command": "test-app"}},
                "architectures": ["all"],
                "assumes": ["command-chain"],
                "base": "core",
                "confinement": "strict",
                "environment": {"TESTING": "1"},
                "epoch": 0,
                "grade": "devel",
                "hooks": {
                    "test-hook": {"command-chain": ["cmd1"], "plugs": ["network"]}
                },
                "layout": {"/target": {"bind": "$SNAP/foo"}},
                "license": "GPL",
                "plugs": {"test-plug": OrderedDict({"interface": "some-value"})},
                "slots": {"test-slot": OrderedDict({"interface": "some-value"})},
                "system-usernames": OrderedDict(
                    {"snap_daemon": OrderedDict({"scope": "shared"})}
                ),
                "title": "test-title",
                "type": "base",
            }
        )

        snap = Snap.from_dict(snap_dict=snap_dict)
        snap.validate()

        self.assertEqual(snap_dict, snap.to_dict())
        self.assertEqual(False, snap.is_passthrough_enabled)
        self.assertEqual(snap_dict["name"], snap.name)
        self.assertEqual(snap_dict["version"], snap.version)
        self.assertEqual(snap_dict["summary"], snap.summary)
        self.assertEqual(snap_dict["description"], snap.description)
        self.assertEqual(snap_dict["apps"]["test-app"], snap.apps["test-app"].to_dict())
        self.assertEqual(snap_dict["architectures"], snap.architectures)
        self.assertEqual(set(snap_dict["assumes"]), snap.assumes)
        self.assertEqual(snap_dict["base"], snap.base)
        self.assertEqual(snap_dict["environment"], snap.environment)
        self.assertEqual(
            snap_dict["hooks"]["test-hook"], snap.hooks["test-hook"].to_dict()
        )
        self.assertEqual(snap_dict["license"], snap.license)
        self.assertEqual(
            snap_dict["plugs"]["test-plug"], snap.plugs["test-plug"].to_yaml_object()
        )
        self.assertEqual(
            snap_dict["slots"]["test-slot"], snap.slots["test-slot"].to_yaml_object()
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

        snap = Snap.from_dict(snap_dict=snap_dict)
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

        snap = Snap.from_dict(snap_dict=snap_dict)
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

        snap = Snap.from_dict(snap_dict=snap_dict)
        snap.validate()

        self.assertEqual(True, snap.is_passthrough_enabled)

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

        snap = Snap.from_dict(snap_dict=snap_dict)
        snap.validate()

        self.assertEqual(True, snap.is_passthrough_enabled)

    def test_from_file(self):
        snap_yaml = dedent(
            """
            name: test-name
            version: "1.0"
            summary: test-summary
            description: test-description
            base: core18
            architectures:
            - amd64
            assumes:
            - snapd2.39
            confinement: classic
            grade: devel
            apps:
              test-app:
                command: test-command
                completer: test-completer
        """
        )

        meta_path = os.path.join(self.path, "meta")
        os.makedirs(meta_path)

        snap_yaml_path = os.path.join(self.path, "meta", "snap.yaml")
        open(snap_yaml_path, "w").write(snap_yaml)

        snap = Snap.from_file(snap_yaml_path)
        snap.validate()

        self.assertEqual("test-name", snap.name)
        self.assertEqual("1.0", snap.version)
        self.assertEqual("test-summary", snap.summary)
        self.assertEqual("test-description", snap.description)
        self.assertEqual(
            OrderedDict({"command": "test-command", "completer": "test-completer"}),
            snap.apps["test-app"].to_dict(),
        )
        self.assertEqual(["amd64"], snap.architectures)
        self.assertEqual({"snapd2.39"}, snap.assumes)
        self.assertEqual("core18", snap.base)
        self.assertEqual("classic", snap.confinement)
        self.assertEqual("devel", snap.grade)

    def test_to_file(self):
        # Ordering matters for verifying the YAML.
        snap_yaml = dedent(
            """
            name: test-name
            version: '1.0'
            summary: test-summary
            description: test-description
            apps:
              test-app:
                command: test-command
                completer: test-completer
            architectures:
            - amd64
            assumes:
            - snapd2.39
            base: core18
            confinement: classic
            grade: devel
        """
        )

        meta_path = os.path.join(self.path, "meta")
        os.makedirs(meta_path)

        snap_yaml_path = os.path.join(self.path, "meta", "snap.yaml")
        open(snap_yaml_path, "w").write(snap_yaml)

        snap = Snap.from_file(snap_yaml_path)
        snap.validate()

        # Write snap yaml.
        snap.write_snap_yaml(path=snap_yaml_path)

        # Read snap yaml.
        written_snap_yaml = open(snap_yaml_path, "r").read()

        # Compare stripped versions (to remove leading/trailing newlines).
        self.assertEqual(snap_yaml.strip(), written_snap_yaml.strip())

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

        snap = Snap.from_dict(snap_dict=snap_dict)
        snap.validate()

        patcher = mock.patch("snapcraft.internal.common.get_installed_snap_path")
        mock_core_path = patcher.start()
        mock_core_path.return_value = self.path
        self.addCleanup(patcher.stop)

        meta_path = os.path.join(self.path, "meta")
        os.makedirs(meta_path)

        snap_yaml_path = os.path.join(meta_path, "snap.yaml")
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

        snap = Snap.from_dict(snap_dict=snap_dict)
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

        snap = Snap.from_dict(snap_dict=snap_dict)
        snap.validate()

        # Write snap yaml.
        snap_yaml_path = os.path.join(self.path, "snap.yaml")
        snap.write_snap_yaml(path=snap_yaml_path)

        # Read snap yaml.
        written_snap_yaml = open(snap_yaml_path, "r").read()

        self.assertEqual(snap_dict, snap.to_dict())
        self.assertFalse("base" in written_snap_yaml)

    def test_write_snap_yaml_with_base_core18(self):
        snap_dict = OrderedDict(
            {
                "name": "snap-test",
                "version": "snap-version",
                "summary": "snap-summary",
                "description": "snap-description",
                "base": "core18",
                "grade": "devel",
            }
        )

        snap = Snap.from_dict(snap_dict=snap_dict)
        snap.validate()

        # Write snap yaml.
        snap_yaml_path = os.path.join(self.path, "snap.yaml")
        snap.write_snap_yaml(path=snap_yaml_path)

        # Read snap yaml.
        written_snap_yaml = open(snap_yaml_path, "r").read()

        self.assertEqual(snap_dict, snap.to_dict())
        self.assertTrue("base" in written_snap_yaml)


class YAMLComparisons(testscenarios.WithScenarios, integration.TestCase):
    scenarios = [
        (
            "slot-all-forms",
            dict(
                snapcraft_yaml=dedent(
                    """
            name: test-name
            version: '1.0'
            summary: test-summary
            description: test-description
            apps:
              test-app:
                command: test-command
            architectures:
            - amd64
            base: core18
            confinement: classic
            grade: devel
            slots:
              long-form:
                interface: content
                content: explicit-content
                read:
                - /
              short-form: interface-name
              shortest-form:
        """
                ),
                snap_yaml=dedent(
                    """
            name: test-name
            version: '1.0'
            summary: test-summary
            description: test-description
            apps:
              test-app:
                command: test-command
            architectures:
            - amd64
            base: core18
            confinement: classic
            grade: devel
            slots:
              long-form:
                interface: content
                content: explicit-content
                read:
                - /
              short-form: interface-name
              shortest-form: null
        """
                ),
            ),
        ),
        (
            "plug-all-forms",
            dict(
                snapcraft_yaml=dedent(
                    """
            name: test-name
            version: '1.0'
            summary: test-summary
            description: test-description
            apps:
              test-app:
                command: test-command
            architectures:
            - amd64
            base: core18
            confinement: classic
            grade: devel
            plugs:
              long-form:
                interface: content
                target: $SNAP/data-dir
                default-provider: some-provider
              short-form: interface-name
              shortest-form:
        """
                ),
                snap_yaml=dedent(
                    """
            name: test-name
            version: '1.0'
            summary: test-summary
            description: test-description
            apps:
              test-app:
                command: test-command
            architectures:
            - amd64
            base: core18
            confinement: classic
            grade: devel
            plugs:
              long-form:
                interface: content
                target: $SNAP/data-dir
                default-provider: some-provider
              short-form: interface-name
              shortest-form: null
        """
                ),
            ),
        ),
    ]

    def setUp(self):
        super().setUp()

    def test_conversions(self):
        # Ordering matters for verifying the YAML.
        snapcraft_yaml_path = os.path.join(self.path, "snapcraft.yaml")
        with open(snapcraft_yaml_path, "w") as f:
            f.write(self.snapcraft_yaml)

        snap = Snap.from_file(snapcraft_yaml_path)
        snap.validate()

        # Write snap yaml.
        snap_yaml_path = os.path.join(self.path, "snap.yaml")
        snap.write_snap_yaml(path=snap_yaml_path)

        # Read snap yaml.
        with open(snap_yaml_path, "r") as f:
            written_snap_yaml = f.read()

        # Compare stripped versions (to remove leading/trailing newlines).
        self.assertEqual(self.snap_yaml.strip(), written_snap_yaml.strip())
