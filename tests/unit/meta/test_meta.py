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

import contextlib
import logging
import os
import stat
import textwrap
from unittest.mock import patch

import fixtures
import testtools
from testtools.matchers import (
    Annotate,
    Contains,
    DirExists,
    Equals,
    FileContains,
    FileExists,
    HasLength,
    Not,
)

from snapcraft import extractors, yaml_utils
from snapcraft.internal import errors, project_loader, states
from snapcraft.internal.meta import _snap_packaging
from snapcraft.internal.meta import errors as meta_errors
from snapcraft.project import Project
from tests import fixture_setup, unit


class CreateBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.config_data = {
            "architectures": [{"build-on": "all", "run-on": "amd64"}],
            "name": "my-package",
            "base": "core18",
            "version": "1.0",
            "description": "my description",
            "summary": "my summary",
            "confinement": "devmode",
            "environment": {"GLOBAL": "y"},
            "parts": {"test-part": {"plugin": "nil"}},
        }

        self.snapcraft_yaml_file_path = os.path.join("snap", "snapcraft.yaml")
        # Ensure the ensure snapcraft.yaml method has something to copy.
        _create_file(self.snapcraft_yaml_file_path)

    def generate_meta_yaml(
        self, *, build=False, actual_prime_dir=None, snapcraft_yaml_file_path=None
    ):
        if snapcraft_yaml_file_path is None:
            snapcraft_yaml_file_path = self.snapcraft_yaml_file_path
        os.makedirs("snap", exist_ok=True)
        with open(snapcraft_yaml_file_path, "w") as f:
            f.write(yaml_utils.dump(self.config_data))

        self.project = Project(snapcraft_yaml_file_path=snapcraft_yaml_file_path)
        if actual_prime_dir is not None:
            self.project._prime_dir = actual_prime_dir

        self.meta_dir = os.path.join(self.project.prime_dir, "meta")
        self.hooks_dir = os.path.join(self.meta_dir, "hooks")
        self.snap_yaml = os.path.join(self.meta_dir, "snap.yaml")

        self.config = project_loader.load_config(project=self.project)
        if build:
            for part in self.config.parts.all_parts:
                part.pull()
                part.build()
                part.stage()
                part.prime()

        _snap_packaging.create_snap_packaging(self.config)

        self.assertTrue(os.path.exists(self.snap_yaml), "snap.yaml was not created")

        with open(self.snap_yaml) as f:
            return yaml_utils.load(f)


class CreateTestCase(CreateBaseTestCase):
    def test_create_meta(self):
        y = self.generate_meta_yaml()

        expected = {
            "architectures": ["amd64"],
            "confinement": "devmode",
            "grade": "stable",
            "description": "my description",
            "environment": {"GLOBAL": "y"},
            "summary": "my summary",
            "name": "my-package",
            "base": "core18",
            "version": "1.0",
        }

        self.assertThat(y, Equals(expected))

    def test_create_meta_with_core_as_base(self):
        self.config_data["base"] = "core"
        self.fake_snapd.snaps_result = [
            dict(name="core", channel="stable", revision="10")
        ]

        y = self.generate_meta_yaml()

        self.assertThat(y, Not(Contains("base")))

    def test_create_meta_default_architecture(self):
        del self.config_data["architectures"]

        y = self.generate_meta_yaml()

        expected = {
            "architectures": [self.project.deb_arch],
            "confinement": "devmode",
            "grade": "stable",
            "description": "my description",
            "environment": {"GLOBAL": "y"},
            "summary": "my summary",
            "name": "my-package",
            "base": "core18",
            "version": "1.0",
        }

        self.assertThat(y, Equals(expected))

    def test_create_meta_with_epoch(self):
        self.config_data["epoch"] = "1*"

        y = self.generate_meta_yaml()
        self.assertTrue(
            "epoch" in y, 'Expected "epoch" property to be copied into snap.yaml'
        )
        self.assertThat(y["epoch"], Equals("1*"))

    def test_create_meta_with_title(self):
        self.config_data["title"] = "The Title"

        y = self.generate_meta_yaml()
        self.assertThat(y, Contains("title"))
        self.assertThat(y["title"], Equals("The Title"))

    def test_create_meta_with_license(self):
        self.config_data["license"] = "MIT"

        y = self.generate_meta_yaml()
        self.assertThat(y, Contains("license"))
        self.assertThat(y["license"], Equals("MIT"))

    def test_create_meta_with_assumes(self):
        self.config_data["assumes"] = ["feature1", "feature2"]

        y = self.generate_meta_yaml()
        self.assertTrue(
            "assumes" in y, 'Expected "assumes" property to be copied into snap.yaml'
        )
        self.assertThat(y["assumes"], Equals(["feature1", "feature2"]))

    def test_create_meta_command_chain_with_assumes(self):
        self.config_data["assumes"] = ["feature1", "feature2"]
        self.config_data["apps"] = {"app": {"command": "foo", "command-chain": ["bar"]}}
        _create_file(os.path.join(self.prime_dir, "foo"), executable=True)
        _create_file(os.path.join(self.prime_dir, "bar"), executable=True)

        y = self.generate_meta_yaml()
        self.assertTrue(
            "assumes" in y, 'Expected "assumes" property to be copied into snap.yaml'
        )
        self.assertThat(
            y["assumes"], Equals(sorted(["feature1", "feature2", "command-chain"]))
        )

    def test_create_gadget_meta_with_gadget_yaml(self):
        gadget_yaml = "stub entry: stub value"
        _create_file("gadget.yaml", content=gadget_yaml)

        self.config_data["type"] = "gadget"

        self.generate_meta_yaml()

        expected_gadget = os.path.join(self.meta_dir, "gadget.yaml")
        self.assertTrue(os.path.exists(expected_gadget))

        self.assertThat(expected_gadget, FileContains(gadget_yaml))

    def test_create_gadget_meta_with_missing_gadget_yaml_raises_error(self):
        self.config_data["type"] = "gadget"

        self.assertRaises(errors.MissingGadgetError, self.generate_meta_yaml)

    def test_create_kernel_meta_with_kernel_yaml(self):
        kernel_yaml = "stub entry: stub value"
        _create_file("kernel.yaml", content=kernel_yaml)

        self.config_data["type"] = "kernel"
        self.config_data["build-base"] = "core20"
        self.config_data.pop("base")

        self.generate_meta_yaml()

        expected_kernel = os.path.join(self.meta_dir, "kernel.yaml")
        self.assertTrue(os.path.exists(expected_kernel))

        self.assertThat(expected_kernel, FileContains(kernel_yaml))

    def test_create_meta_with_declared_icon(self):
        _create_file(os.path.join(os.curdir, "my-icon.png"))
        self.config_data["icon"] = "my-icon.png"

        y = self.generate_meta_yaml()

        self.assertThat(os.path.join(self.meta_dir, "gui", "icon.png"), FileExists())

        self.assertFalse("icon" in y, "icon found in snap.yaml {}".format(y))

    def test_create_meta_with_declared_icon_with_dots(self):
        _create_file("com.my.icon.png")
        self.config_data["icon"] = "com.my.icon.png"

        y = self.generate_meta_yaml()

        self.assertThat(os.path.join(self.meta_dir, "gui", "icon.png"), FileExists())

        self.assertFalse("icon" in y, "icon found in snap.yaml {}".format(y))

    def test_create_meta_with_declared_icon_in_parent_dir(self):
        _create_file("my-icon.png")
        builddir = os.path.join(os.curdir, "subdir")
        os.mkdir(builddir)
        os.chdir(builddir)
        self.config_data["icon"] = "../my-icon.png"

        y = self.generate_meta_yaml()

        self.assertThat(os.path.join(self.meta_dir, "gui", "icon.png"), FileExists())

        self.assertFalse("icon" in y, "icon found in snap.yaml {}".format(y))

    def test_create_meta_with_declared_icon_and_setup(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        gui_path = os.path.join("setup", "gui")
        os.makedirs(gui_path)
        setup_icon_content = "setup icon"
        _create_file(os.path.join(gui_path, "icon.png"), content=setup_icon_content)

        declared_icon_content = "declared icon"
        _create_file("my-icon.png", content=declared_icon_content)
        self.config_data["icon"] = "my-icon.png"

        y = self.generate_meta_yaml()

        expected_icon = os.path.join(self.meta_dir, "gui", "icon.png")
        self.assertTrue(
            os.path.exists(expected_icon), "icon.png was not setup correctly"
        )
        self.assertThat(expected_icon, FileContains(declared_icon_content))

        self.assertFalse("icon" in y, "icon found in snap.yaml {}".format(y))

        # Check for the correct deprecation message.
        self.assertIn(
            "Assets in 'setup/gui' should now be placed in 'snap/gui'.",
            fake_logger.output,
        )
        self.assertIn(
            "See http://snapcraft.io/docs/deprecation-notices/dn3", fake_logger.output
        )

    @patch("os.link", side_effect=OSError("Invalid cross-device link"))
    def test_create_meta_with_declared_icon_when_os_link_raises(self, link_mock):
        _create_file(os.path.join(os.curdir, "my-icon.png"))
        self.config_data["icon"] = "my-icon.png"

        y = self.generate_meta_yaml()

        self.assertThat(os.path.join(self.meta_dir, "gui", "icon.png"), FileExists())

        self.assertFalse("icon" in y, "icon found in snap.yaml {}".format(y))

    def test_create_meta_with_declared_icon_and_setup_ran_twice_ok(self):
        gui_path = os.path.join("setup", "gui")
        os.makedirs(gui_path)
        icon_content = "setup icon"
        _create_file(os.path.join(gui_path, "icon.png"), content=icon_content)

        _create_file("my-icon.png")
        self.config_data["icon"] = "my-icon.png"

        self.generate_meta_yaml()

        # Running again should be good
        _snap_packaging.create_snap_packaging(self.config)

    def test_create_meta_with_icon_in_setup(self):
        gui_path = os.path.join("setup", "gui")
        os.makedirs(gui_path)
        icon_content = "setup icon"
        _create_file(os.path.join(gui_path, "icon.png"), content=icon_content)

        y = self.generate_meta_yaml()

        expected_icon = os.path.join(self.meta_dir, "gui", "icon.png")
        self.assertThat(expected_icon, FileContains(icon_content))

        self.assertFalse("icon" in y, "icon found in snap.yaml {}".format(y))

    def test_version_script(self):
        self.config_data["version-script"] = "echo 10.1-devel"

        y = self.generate_meta_yaml()

        self.assertThat(y["version"], Equals("10.1-devel"))

    def test_version_script_exits_bad(self):
        self.config_data["version-script"] = "exit 1"

        with testtools.ExpectedException(meta_errors.CommandError):
            self.generate_meta_yaml()

    def test_version_script_with_no_output(self):
        self.config_data["version-script"] = "echo"

        with testtools.ExpectedException(meta_errors.CommandError):
            self.generate_meta_yaml()

    def test_layout(self):
        layout = {"/target": {"bind": "$SNAP/foo"}}
        self.config_data["layout"] = layout

        y = self.generate_meta_yaml()

        self.assertThat(y["layout"], Equals(layout))

    def test_adapter_full_with_command_chain(self):
        self.config_data["apps"] = {"app": {"command": "foo", "command-chain": ["bar"]}}
        _create_file(os.path.join(self.prime_dir, "foo"), executable=True)
        _create_file(os.path.join(self.prime_dir, "bar"), executable=True)

        y = self.generate_meta_yaml()

        self.expectThat(y["apps"]["app"]["command"], Equals("foo"))
        self.expectThat(
            y["apps"]["app"]["command-chain"],
            Equals([os.path.join("snap", "command-chain", "snapcraft-runner"), "bar"]),
        )
        self.assertThat(y["assumes"], Equals(["command-chain"]))

    def test_adapter_assumes_command_chain(self):
        self.config_data["apps"] = {"app": {"command": "foo"}}
        _create_file(os.path.join(self.prime_dir, "foo"), executable=True)

        y = self.generate_meta_yaml()

        self.expectThat(y["apps"]["app"]["command"], Equals("foo"))
        self.expectThat(
            y["apps"]["app"]["command-chain"],
            Equals([os.path.join("snap", "command-chain", "snapcraft-runner")]),
        )
        self.assertThat(y["assumes"], Equals(["command-chain"]))

    def test_adapter_none(self):
        # Adapter "none" will passthrough command as-is.
        self.config_data["apps"] = {"app": {"adapter": "none", "command": "/foo"}}

        y = self.generate_meta_yaml()

        self.assertThat(y["apps"]["app"]["command"], Equals("/foo"))
        self.assertThat(y["apps"]["app"].get("command-chain"), Equals(None))


class StopModeTestCase(CreateBaseTestCase):

    stop_modes = [
        "sigterm",
        "sigterm-all",
        "sighup",
        "sighup-all",
        "sigusr1",
        "sigusr1-all",
        "sigusr2",
        "sigusr2-all",
    ]

    def test_valid(self):
        for mode in self.stop_modes:
            self.config_data["apps"] = {
                "app1": {"command": "sh", "daemon": "simple", "stop-mode": mode}
            }

            y = self.generate_meta_yaml()

            self.expectThat(y["apps"]["app1"]["stop-mode"], Equals(mode))


class RefreshModeTestCase(CreateBaseTestCase):

    refresh_modes = ["endure", "restart"]

    def test_valid(self):
        for refresh_mode in self.refresh_modes:
            self.config_data["apps"] = {
                "app1": {
                    "command": "sh",
                    "daemon": "simple",
                    "refresh-mode": refresh_mode,
                }
            }

            y = self.generate_meta_yaml()

            self.expectThat(y["apps"]["app1"]["refresh-mode"], Equals(refresh_mode))


class BeforeAndAfterTest(CreateBaseTestCase):
    def test_before_valid(self):
        self.config_data["apps"] = {
            "app-before": {"command": "sh", "daemon": "simple"},
            "app": {"command": "sh", "daemon": "simple", "before": ["app-before"]},
        }

        y = self.generate_meta_yaml()

        self.assertThat(y["apps"]["app"]["before"], Equals(["app-before"]))

    def test_after_valid(self):
        self.config_data["apps"] = {
            "app-after": {"command": "sh", "daemon": "simple"},
            "app": {"command": "sh", "daemon": "simple", "after": ["app-after"]},
        }

        y = self.generate_meta_yaml()

        self.assertThat(y["apps"]["app"]["after"], Equals(["app-after"]))


class PassthroughBaseTestCase(CreateBaseTestCase):
    def setUp(self):
        super().setUp()

        self.config_data = {
            "name": "my-package",
            "base": "core18",
            "version": "1.0",
            "grade": "stable",
            "description": "my description",
            "summary": "my summary",
            "parts": {"test-part": {"plugin": "nil"}},
        }


class PassthroughErrorTestCase(PassthroughBaseTestCase):
    def test_ambiguous_key_fails(self):
        self.config_data["confinement"] = "devmode"
        self.config_data["passthrough"] = {"confinement": "next-generation"}
        raised = self.assertRaises(
            meta_errors.AmbiguousPassthroughKeyError, self.generate_meta_yaml
        )
        self.assertThat(raised.keys, Equals("'confinement'"))

    def test_app_ambiguous_key_fails(self):
        self.config_data["apps"] = {
            "foo": {
                "command": "echo",
                "daemon": "simple",
                "passthrough": {"daemon": "complex"},
            }
        }
        raised = self.assertRaises(
            meta_errors.AmbiguousPassthroughKeyError, self.generate_meta_yaml
        )
        self.assertThat(raised.keys, Equals("'daemon'"))

    def test_hook_ambiguous_key_fails(self):
        self.config_data["hooks"] = {
            "foo": {"plugs": ["network"], "passthrough": {"plugs": ["network"]}}
        }
        raised = self.assertRaises(
            meta_errors.AmbiguousPassthroughKeyError, self.generate_meta_yaml
        )
        self.assertThat(raised.keys, Equals("'plugs'"))

    def test_warn_once_only(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        self.config_data["confinement"] = "devmode"
        self.config_data["passthrough"] = {"foo": "bar", "spam": "eggs"}
        self.config_data["apps"] = {
            "foo": {"command": "echo", "passthrough": {"foo": "bar", "spam": "eggs"}}
        }
        self.config_data["hooks"] = {
            "foo": {"plugs": ["network"], "passthrough": {"foo": "bar", "spam": "eggs"}}
        }
        self.generate_meta_yaml()

        output_lines = fake_logger.output.splitlines()
        self.assertThat(
            output_lines,
            Contains(
                "The 'passthrough' property is being used to propagate "
                "experimental properties to snap.yaml that have not been "
                "validated."
            ),
        )
        self.assertThat(
            len(output_lines),
            Annotate(
                "There were duplicate lines logged.", Equals(len(set(output_lines)))
            ),
        )


class PassthroughPropagateTestCase(PassthroughBaseTestCase):

    cases = [
        (
            "new",
            dict(
                snippet={"passthrough": {"spam": "eggs"}},
                section=None,
                name=None,
                key="spam",
                value="eggs",
            ),
        ),
        (
            "different type",
            dict(
                # This is normally an array of strings
                snippet={"passthrough": {"architectures": "all"}},
                section=None,
                name=None,
                key="architectures",
                value="all",
            ),
        ),
        (
            "with default",
            dict(
                snippet={"passthrough": {"confinement": "next-generation"}},
                section=None,
                name=None,
                key="confinement",
                value="next-generation",
            ),
        ),
        (
            "app, new",
            dict(
                snippet={
                    "apps": {
                        "foo": {"command": "echo", "passthrough": {"spam": "eggs"}}
                    }
                },
                section="apps",
                name="foo",
                key="spam",
                value="eggs",
            ),
        ),
        (
            "app, different type",
            dict(
                snippet={
                    "apps": {
                        "foo": {
                            "command": "echo",
                            # This is normally an array of strings
                            "passthrough": {"aliases": "foo"},
                        }
                    }
                },
                section="apps",
                name="foo",
                key="aliases",
                value="foo",
            ),
        ),
        # Note: There are currently no app properties with defaults
        (
            "hook, new",
            dict(
                snippet={
                    "hooks": {
                        "foo": {"plugs": ["network"], "passthrough": {"spam": "eggs"}}
                    }
                },
                section="hooks",
                name="foo",
                key="spam",
                value="eggs",
            ),
        ),
        (
            "hook, different type",
            dict(
                snippet={
                    "hooks": {
                        "foo": {
                            # This is normally an array of strings
                            "passthrough": {"plugs": "network"}
                        }
                    }
                },
                section="hooks",
                name="foo",
                key="plugs",
                value="network",
            ),
        ),
        # Note: There are currently no hook properties with defaults
    ]

    def assert_passthrough(self, snippet, section, name, key, value):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        self.config_data.update(snippet)
        y = self.generate_meta_yaml()
        if section:
            y = y[section][name]
            self.expectThat(
                y,
                Contains(key),
                "Expected {!r} property to be propagated to snap.yaml".format(key),
            )
            self.expectThat(y[key], Equals(value))
            self.expectThat(
                fake_logger.output,
                Contains(
                    "The 'passthrough' property is being used to propagate "
                    "experimental properties to snap.yaml that have not been "
                    "validated.\n"
                ),
            )

    def test_propagate(self):
        for case in self.cases:
            self.assert_passthrough(**case[1])


class CreateMetadataFromSourceBaseTestCase(CreateBaseTestCase):
    def setUp(self):
        super().setUp()
        self.config_data = {
            "name": "test-name",
            "base": "core18",
            "version": "test-version",
            "summary": "test-summary",
            "description": "test-description",
            "adopt-info": "test-part",
            "parts": {
                "test-part": {
                    "plugin": "dump",
                    "source": ".",
                    "parse-info": ["test-metadata-file"],
                }
            },
            "apps": {"test-app": {"command": "echo"}},
        }
        # Create metadata file
        open("test-metadata-file", "w").close()


class CreateMetadataFromSourceTestCase(CreateMetadataFromSourceBaseTestCase):
    def test_create_metadata_with_missing_parse_info(self):
        del self.config_data["summary"]
        del self.config_data["parts"]["test-part"]["parse-info"]
        raised = self.assertRaises(
            meta_errors.AdoptedPartNotParsingInfo, self.generate_meta_yaml, build=True
        )
        self.assertThat(raised.part, Equals("test-part"))

    def test_create_metadata_with_wrong_adopt_info(self):
        del self.config_data["summary"]
        self.config_data["adopt-info"] = "wrong-part"
        raised = self.assertRaises(
            meta_errors.AdoptedPartMissingError, self.generate_meta_yaml
        )
        self.assertThat(raised.part, Equals("wrong-part"))

    def test_metadata_doesnt_overwrite_specified(self):
        fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(fake_logger)

        def _fake_extractor(file_path, workdir):
            return extractors.ExtractedMetadata(
                summary="extracted summary", description="extracted description"
            )

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        y = self.generate_meta_yaml(build=True)

        # Since both summary and description were specified, neither should be
        # overwritten
        self.assertThat(y["summary"], Equals(self.config_data["summary"]))
        self.assertThat(y["description"], Equals(self.config_data["description"]))

        # Verify that we warn that the YAML took precedence over the extracted
        # metadata for summary and description
        self.assertThat(
            fake_logger.output,
            Contains(
                "The 'description' and 'summary' properties are specified in "
                "adopted info as well as the YAML: taking the properties from the "
                "YAML"
            ),
        )

    def test_metadata_with_unexisting_icon(self):
        def _fake_extractor(file_path, workdir):
            return extractors.ExtractedMetadata(
                icon="test/extracted/unexistent/icon/path"
            )

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        # The meta generation should just ignore the dead path, and not fail.
        self.generate_meta_yaml(build=True)

    def test_metadata_satisfies_required_property(self):
        del self.config_data["summary"]

        def _fake_extractor(file_path, workdir):
            return extractors.ExtractedMetadata(
                summary="extracted summary", description="extracted description"
            )

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        y = self.generate_meta_yaml(build=True)

        # Summary should come from the extracted metadata, while description
        # should not.
        self.assertThat(y["summary"], Equals("extracted summary"))
        self.assertThat(y["description"], Equals(self.config_data["description"]))

    def test_metadata_not_all_properties_satisfied(self):
        del self.config_data["summary"]
        del self.config_data["description"]

        def _fake_extractor(file_path, workdir):
            return extractors.ExtractedMetadata(description="extracted description")

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        # Assert that description has been satisfied by extracted metadata, but
        # summary has not.
        raised = self.assertRaises(
            meta_errors.MissingSnapcraftYamlKeysError,
            self.generate_meta_yaml,
            build=True,
        )
        self.assertThat(raised.keys, Equals("'summary'"))


class CreateWithAssetsTestCase(CreateBaseTestCase):
    def assert_create_meta_with_hook(self, snapcraft_assets_dir):
        hooksdir = os.path.join(snapcraft_assets_dir, "hooks")
        os.makedirs(hooksdir)
        _create_file(os.path.join(hooksdir, "foo"), executable=True)
        _create_file(os.path.join(hooksdir, "bar"), executable=True)
        self.config_data["hooks"] = {"foo": {"plugs": ["plug"]}, "bar": {}}

        y = self.generate_meta_yaml(
            snapcraft_yaml_file_path=os.path.join(
                snapcraft_assets_dir, "snapcraft.yaml"
            )
        )

        self.assertThat(
            y, Contains("hooks"), "Expected generated YAML to contain 'hooks'"
        )

        for hook in ("foo", "bar"):
            meta_dir_hook_path = os.path.join(self.prime_dir, "meta", "hooks", hook)
            snap_dir_hook_path = os.path.join(self.prime_dir, "snap", "hooks", hook)

            for hook_location in (meta_dir_hook_path, snap_dir_hook_path):
                self.assertThat(
                    hook_location,
                    FileExists(),
                    "The {!r} hook was not setup correctly".format(hook),
                )

        for hook in ("foo", "bar"):
            self.assertThat(
                y["hooks"],
                Contains(hook),
                "Expected generated hooks to contain {!r}".format(hook),
            )

        self.assertThat(
            y["hooks"]["foo"],
            Contains("plugs"),
            "Expected generated 'foo' hook to contain 'plugs'",
        )
        self.assertThat(y["hooks"]["foo"]["plugs"], HasLength(1))
        self.assertThat(y["hooks"]["foo"]["plugs"][0], Equals("plug"))
        self.assertThat(
            y["hooks"]["bar"],
            Not(Contains("plugs")),
            "Expected generated 'bar' hook to not contain 'plugs'",
        )

    def test_create_meta_with_hook_in_snap(self):
        self.assert_create_meta_with_hook("snap")

    def test_create_meta_with_hook_in_build_aux(self):
        self.assert_create_meta_with_hook("build-aux/snap")

    def assert_local_is_not_copied_to_snap(self, snapcraft_assets_dir):
        project_local_dir = os.path.join(snapcraft_assets_dir, "local")
        local_file = "file"
        local_subdir_file = os.path.join("dir", "file")

        os.makedirs(os.path.join(project_local_dir, "dir"))
        _create_file(os.path.join(project_local_dir, local_file))
        _create_file(os.path.join(project_local_dir, local_subdir_file))

        self.generate_meta_yaml(
            snapcraft_yaml_file_path=os.path.join(
                snapcraft_assets_dir, "snapcraft.yaml"
            )
        )

        prime_local_dir = os.path.join(self.prime_dir, "snap", "local")
        self.assertThat(os.path.join(prime_local_dir, local_file), Not(FileExists()))
        self.assertThat(
            os.path.join(prime_local_dir, local_subdir_file), Not(FileExists())
        )

    def test_local_is_not_copied_to_snap_with_snap(self):
        self.assert_create_meta_with_hook("snap")

    def test_local_is_not_copied_to_snap_with_build_aux(self):
        self.assert_create_meta_with_hook("build-aux/snap")


class MetadataFromSourceWithIconFileTestCase(CreateMetadataFromSourceBaseTestCase):
    def assert_no_overwrite(self, snapcraft_assets_dir, directory, file_name):
        os.makedirs(directory)
        icon_content = "setup icon"
        _create_file(os.path.join(directory, file_name), content=icon_content)

        def _fake_extractor(file_path, workdir):
            return extractors.ExtractedMetadata(
                icon="test/extracted/unexistent/icon/path"
            )

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        self.generate_meta_yaml(
            build=True,
            snapcraft_yaml_file_path=os.path.join(
                snapcraft_assets_dir, "snapcraft.yaml"
            ),
        )

        expected_icon = os.path.join(self.meta_dir, "gui", file_name)
        self.assertThat(expected_icon, FileContains(icon_content))

    def test_setup_gui_svg(self):
        self.assert_no_overwrite("snap", "setup/gui", "icon.svg")

    def test_snap_gui_svg(self):
        self.assert_no_overwrite("snap", "snap/gui", "icon.svg")

    def test_build_aux_gui_svg(self):
        self.assert_no_overwrite("build-aux/snap", "build-aux/snap/gui", "icon.svg")

    def test_setup_gui_png(self):
        self.assert_no_overwrite("snap", "setup/gui", "icon.png")

    def test_snap_gui_png(self):
        self.assert_no_overwrite("snap", "snap/gui", "icon.png")

    def test_build_aux_gui_png(self):
        self.assert_no_overwrite("build-aux/snap", "build-aux/snap/gui", "icon.png")


class MetadataFromSourceWithDesktopFileTestCase(CreateMetadataFromSourceBaseTestCase):
    def assert_no_overwrite(self, snapcraft_assets_dir, directory):
        os.makedirs(directory)
        desktop_content = "setup desktop"
        _create_file(
            os.path.join(directory, "test-app.desktop"), content=desktop_content
        )

        def _fake_extractor(file_path, workdir):
            return extractors.ExtractedMetadata(
                desktop_file_paths=[
                    "usr/share/applications/com.example.test/app.desktop"
                ]
            )

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        self.generate_meta_yaml(
            build=True,
            snapcraft_yaml_file_path=os.path.join(
                snapcraft_assets_dir, "snapcraft.yaml"
            ),
        )

        expected_desktop = os.path.join(self.meta_dir, "gui", "test-app.desktop")
        self.assertThat(expected_desktop, FileContains(desktop_content))

    def test_setup_gui(self):
        self.assert_no_overwrite("snap", "setup/gui")

    def test_snap_gui(self):
        self.assert_no_overwrite("snap", "snap/gui")

    def test_build_aux_gui(self):
        self.assert_no_overwrite("build-aux/snap", "build-aux/snap/gui")


class ScriptletsMetadataTestCase(CreateMetadataFromSourceBaseTestCase):
    def assert_scriptlets_satisfy_required_property(
        self, keyword, original, value, setter
    ):
        with contextlib.suppress(KeyError):
            del self.config_data[keyword]

        del self.config_data["parts"]["test-part"]["parse-info"]
        self.config_data["parts"]["test-part"][
            "override-prime"
        ] = "snapcraftctl {} {}".format(setter, value)

        generated = self.generate_meta_yaml(build=True)

        self.assertThat(generated[keyword], Equals(value))

    def assert_scriptlets_no_overwrite_existing_property(
        self, keyword, original, value, setter
    ):
        self.config_data[keyword] = original
        fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(fake_logger)

        del self.config_data["parts"]["test-part"]["parse-info"]
        self.config_data["parts"]["test-part"][
            "override-prime"
        ] = "snapcraftctl {} {}".format(setter, value)

        generated = self.generate_meta_yaml(build=True)

        self.assertThat(generated[keyword], Equals(original))

        # Since the specified version took precedence over the scriptlet-set
        # version, verify that we warned
        self.assertThat(
            fake_logger.output,
            Contains(
                "The {!r} property is specified in adopted info as well as "
                "the YAML: taking the property from the YAML".format(keyword)
            ),
        )

    def assert_scriptlets_overwrite_extracted_metadata(
        self, keyword, original, value, setter
    ):
        with contextlib.suppress(KeyError):
            del self.config_data[keyword]

        self.config_data["parts"]["test-part"][
            "override-build"
        ] = "snapcraftctl build && snapcraftctl {} {}".format(setter, value)

        def _fake_extractor(file_path, workdir):
            return extractors.ExtractedMetadata(**{keyword: "extracted-value"})

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        generated = self.generate_meta_yaml(build=True)

        self.assertThat(generated[keyword], Equals(value))

    def assert_scriptlets_overwrite_extracted_metadata_regardless_of_order(
        self, keyword, original, value, setter
    ):
        with contextlib.suppress(KeyError):
            del self.config_data[keyword]

        self.config_data["parts"]["test-part"][
            "override-pull"
        ] = "snapcraftctl {} {} && snapcraftctl pull".format(setter, value)

        def _fake_extractor(file_path, workdir):
            return extractors.ExtractedMetadata(**{keyword: "extracted-value"})

        self.useFixture(fixture_setup.FakeMetadataExtractor("fake", _fake_extractor))

        generated = self.generate_meta_yaml(build=True)

        self.assertThat(generated[keyword], Equals(value))

    def test_scriptlets_satisfy_required_property_for_grade(self):
        self.assert_scriptlets_satisfy_required_property(
            "grade", "stable", "devel", "set-grade"
        )

    def test_scriptlets_overwrite_extracted_metadata_for_grade(self):
        self.assert_scriptlets_overwrite_extracted_metadata(
            "grade", "stable", "devel", "set-grade"
        )

    def test_scriptlets_no_overwrite_existing_property_for_grade(self):
        self.assert_scriptlets_no_overwrite_existing_property(
            "grade", "stable", "devel", "set-grade"
        )

    def test_scriptlets_overwrite_extracted_metadata_regardless_of_order_for_grade(
        self,
    ):
        self.assert_scriptlets_overwrite_extracted_metadata_regardless_of_order(
            "grade", "stable", "devel", "set-grade"
        )

    def test_scriptlets_satisfy_required_property_for_version(self):
        self.assert_scriptlets_satisfy_required_property(
            "version", "original-version", "new-version", "set-version"
        )

    def test_scriptlets_overwrite_extracted_metadata_for_version(self):
        self.assert_scriptlets_overwrite_extracted_metadata(
            "version", "original-version", "new-version", "set-version"
        )

    def test_scriptlets_no_overwrite_existing_property_for_version(self):
        self.assert_scriptlets_no_overwrite_existing_property(
            "version", "original-version", "new-version", "set-version"
        )

    def test_scriptlets_overwrite_extracted_metadata_regardless_of_order_for_version(
        self,
    ):
        self.assert_scriptlets_overwrite_extracted_metadata_regardless_of_order(
            "version", "original-version", "new-version", "set-version"
        )


class WriteSnapDirectoryTestCase(CreateBaseTestCase):
    def test_write_snap_directory(self):
        # Setup a snap directory containing a few things.
        _create_file(os.path.join(self.snap_dir, "snapcraft.yaml"))
        _create_file(os.path.join(self.snap_dir, "hooks", "test-hook"), executable=True)

        # Now write the snap directory, and verify everything was migrated, as
        # well as the hook making it into meta/.
        self.generate_meta_yaml()
        prime_snap_dir = os.path.join(self.prime_dir, "snap")

        self.assertThat(
            os.path.join(prime_snap_dir, "hooks", "test-hook"), FileExists()
        )
        self.assertThat(os.path.join(self.hooks_dir, "test-hook"), FileExists())

        # The hook should be empty, because the one in snap/hooks is empty, and
        # no wrapper is generated (i.e. that hook is copied to both locations).
        self.assertThat(os.path.join(self.hooks_dir, "test-hook"), FileContains(""))

    def test_nothing_to_write(self):
        # Setup a snap directory containing just a snapcraft.yaml, nothing else
        _create_file(os.path.join(self.snap_dir, "snapcraft.yaml"))

        # Now generate the metadata, and verify that no snap directory was
        # created
        self.generate_meta_yaml()
        self.assertThat(
            os.path.join(self.prime_dir, "snap"),
            Not(DirExists()),
            "Expected snap directory to NOT be created",
        )

    def test_snap_hooks_overwrite_part_hooks(self):
        # Setup a prime/snap directory containing a hook.
        part_hook = os.path.join(self.prime_dir, "snap", "hooks", "test-hook")
        _create_file(part_hook, content="from part", executable=True)

        # Setup a snap directory containing the same hook
        snap_hook = os.path.join(self.snap_dir, "hooks", "test-hook")
        _create_file(snap_hook, content="from snap", executable=True)

        # Now write the snap directory, and verify that the snap hook overwrote
        # the part hook in both prime/snap/hooks and prime/meta/hooks.
        self.generate_meta_yaml()
        prime_snap_dir = os.path.join(self.prime_dir, "snap")
        self.assertThat(
            os.path.join(prime_snap_dir, "hooks", "test-hook"), FileExists()
        )
        self.assertThat(os.path.join(self.hooks_dir, "test-hook"), FileExists())

        # Both hooks in snap/hooks and meta/hooks should contain 'from snap' as
        # that one should have overwritten the other (and its wrapper).
        self.assertThat(
            os.path.join(self.prime_dir, "snap", "hooks", "test-hook"),
            FileContains("from snap"),
        )
        self.assertThat(
            os.path.join(self.prime_dir, "meta", "hooks", "test-hook"),
            FileContains("from snap"),
        )

    def test_hooks_with_command_chain_write_stub(self):
        self.config_data["hooks"] = {
            "configure": {"command-chain": ["fake-hook-command-chain"]}
        }

        self.generate_meta_yaml()

        self.assertThat(os.path.join(self.hooks_dir, "configure"), FileExists())

        # The hook should be empty, because the one in snap/hooks is empty, and
        # no wrapper is generated (i.e. that hook is copied to both locations).
        self.assertThat(
            os.path.join(self.hooks_dir, "configure"), FileContains("#!/bin/sh\n")
        )

    def test_snap_hooks_stubs_not_created_stubs_from_command_chain(self):
        self.config_data["hooks"] = {
            "install": {"command-chain": ["fake-hook-command-chain"]}
        }

        # Setup a prime/snap directory containing a hook.
        part_hook = os.path.join(self.prime_dir, "snap", "hooks", "install")
        _create_file(part_hook, content="from part", executable=True)

        # Now write the snap directory, and verify that the snap hook overwrote
        # the part hook in both prime/snap/hooks and prime/meta/hooks.
        self.generate_meta_yaml()

        self.assertThat(
            os.path.join(self.hooks_dir, "install"),
            FileContains(
                textwrap.dedent(
                    """\
            #!/bin/sh
            export PATH="$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin${PATH:+:$PATH}"
            export LD_LIBRARY_PATH="$SNAP_LIBRARY_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            exec "$SNAP/snap/hooks/install" "$@"
            """
                )
            ),
        )

    def test_snap_hooks_not_executable_chmods(self):
        real_chmod = os.chmod

        def chmod_noop(name, *args, **kwargs):
            # Simulate a source filesystem that doesn't allow changing permissions
            if name.startswith("snap"):
                pass
            else:
                real_chmod(name, *args, **kwargs)

        self.useFixture(fixtures.MonkeyPatch("os.chmod", chmod_noop))

        # Setup a snap directory containing a few things.
        _create_file(os.path.join(self.snap_dir, "snapcraft.yaml"))
        _create_file(os.path.join(self.snap_dir, "hooks", "test-hook"))

        # Now write the snap directory.
        self.generate_meta_yaml()

        # Ensure the file is executable
        self.assertThat(os.path.join(self.hooks_dir, "test-hook"), FileExists())
        self.assertTrue(
            os.stat(os.path.join(self.hooks_dir, "test-hook")).st_mode & stat.S_IEXEC
        )

    def test_snap_hooks_not_does_not_fail_on_symlink(self):
        # Setup a snap directory containing a few things.
        _create_file(os.path.join(self.snap_dir, "snapcraft.yaml"))
        _create_file(os.path.join(self.snap_dir, "hooks", "test-hook"))
        # Create a working symlink
        os.symlink(
            "test-hook", os.path.join(self.snap_dir, "hooks", "test-hook-symlink")
        )

        # Now write the snap directory.
        self.generate_meta_yaml()

        # Ensure the file is executable
        self.assertThat(os.path.join(self.hooks_dir, "test-hook"), FileExists())
        test_hook_stat = os.stat(
            os.path.join(self.hooks_dir, "test-hook"), follow_symlinks=False
        )
        test_hook_symlink_stat = os.stat(
            os.path.join(self.hooks_dir, "test-hook-symlink"), follow_symlinks=False
        )

        self.assertThat(test_hook_symlink_stat.st_ino, Equals(test_hook_stat.st_ino))


class GenerateHookWrappersTestCase(CreateBaseTestCase):
    def test_generate_hook_wrappers(self):
        # Set up the prime directory to contain a few hooks in snap/hooks
        snap_hooks_dir = os.path.join(self.prime_dir, "snap", "hooks")
        hook1_path = os.path.join(snap_hooks_dir, "test-hook1")
        hook2_path = os.path.join(snap_hooks_dir, "test-hook2")

        for path in (hook1_path, hook2_path):
            _create_file(path, executable=True)

        # Now generate hook wrappers, and verify that they're correct
        self.generate_meta_yaml()
        for hook in ("test-hook1", "test-hook2"):
            hook_path = os.path.join(self.hooks_dir, hook)
            self.assertThat(hook_path, FileExists())
            self.assertThat(hook_path, unit.IsExecutable())

            # The hook in meta/hooks should exec the one in snap/hooks, as it's
            # a wrapper generated by snapcraft.
            self.assertThat(
                hook_path,
                FileContains(
                    matcher=Contains('exec "$SNAP/snap/hooks/{}"'.format(hook))
                ),
            )

    @patch("snapcraft.internal.project_loader._config.Config.snap_env")
    def test_generated_hook_wrappers_include_environment(self, mock_snap_env):
        mock_snap_env.return_value = ["PATH={}/foo".format(self.prime_dir)]

        # Set up the prime directory to contain a hook in snap/hooks as well as
        # one in meta/hooks
        snap_hook = os.path.join(self.prime_dir, "snap", "hooks", "snap-hook")
        meta_hook = os.path.join(self.prime_dir, "meta", "hooks", "meta-hook")

        for path in (snap_hook, meta_hook):
            _create_file(path, executable=True, content=path)

        # Now generate hook wrappers
        self.generate_meta_yaml()

        # Verify that the hook already in meta was unchanged (no environment)
        final_meta_hook = os.path.join(self.hooks_dir, "meta-hook")
        self.assertThat(final_meta_hook, FileExists())
        self.assertThat(final_meta_hook, unit.IsExecutable())
        self.assertThat(final_meta_hook, FileContains(meta_hook))

        # Verify that the snap hook was unchanged
        self.assertThat(snap_hook, FileExists())
        self.assertThat(snap_hook, unit.IsExecutable())
        self.assertThat(snap_hook, FileContains(snap_hook))

        # Verify that the snap hook got a wrapper generated for it with a full
        # environment
        final_snap_hook = os.path.join(self.hooks_dir, "snap-hook")
        self.assertThat(final_snap_hook, FileExists())
        self.assertThat(final_snap_hook, unit.IsExecutable())
        expected = textwrap.dedent(
            """\
            #!/bin/sh
            export PATH=$SNAP/foo
            export LD_LIBRARY_PATH="$SNAP_LIBRARY_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            exec "$SNAP/snap/hooks/snap-hook" "$@"
        """
        )

        self.assertThat(final_snap_hook, FileContains(expected))

    def test_generate_hook_wrappers_not_executable_chmods(self):
        # Set up the prime directory to contain a hook in snap/hooks that is
        # not executable.
        snap_hooks_dir = os.path.join(self.prime_dir, "snap", "hooks")
        _create_file(os.path.join(snap_hooks_dir, "test-hook"))

        # Now generate hook wrappers.
        self.generate_meta_yaml()

        # Ensure the file is executable
        self.assertThat(os.path.join(self.hooks_dir, "test-hook"), FileExists())
        self.assertTrue(
            os.stat(os.path.join(self.hooks_dir, "test-hook")).st_mode & stat.S_IEXEC
        )


class TestConfinement(CreateBaseTestCase):
    def test_strict(self):
        self.config_data["confinement"] = "strict"

        self.assertThat(self.generate_meta_yaml()["confinement"], Equals("strict"))

    def test_devmode(self):
        self.config_data["confinement"] = "devmode"

        self.assertThat(self.generate_meta_yaml()["confinement"], Equals("devmode"))

    def test_no_confinement(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        del self.config_data["confinement"]

        # Ensure confinement defaults to strict if not specified. Also
        # verify that a warning is printed
        self.assertThat(self.generate_meta_yaml()["confinement"], Equals("strict"))
        self.assertThat(
            fake_logger.output,
            Contains("'confinement' property not specified: defaulting to 'strict'"),
        )


class TestRootEnvironmentLibraryPathWarnings(CreateBaseTestCase):
    def setUp(self):
        super().setUp()

        self.config_data["grade"] = "stable"

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

    def assert_warnings(self):
        self.generate_meta_yaml()

        self.assertThat(
            self.fake_logger.output,
            Contains(
                "CVE-2020-27348: A potentially empty LD_LIBRARY_PATH has been set for environment "
                "in '.'. The current working directory will be added to the library path if empty. "
                "This can cause unexpected libraries to be loaded."
            ),
        )

    def assert_no_warnings(self):
        self.generate_meta_yaml()

        self.assertThat(
            self.fake_logger.output,
            Not(
                Contains(
                    "CVE-2020-27348: A potentially empty LD_LIBRARY_PATH has been set for environment "
                    "in '.'. The current working directory will be added to the library path if empty. "
                    "This can cause unexpected libraries to be loaded."
                )
            ),
        )

    def test_root_ld_library_path(self):
        self.config_data["environment"] = {"LD_LIBRARY_PATH": "/foo:$LD_LIBRARY_PATH"}

        self.assert_warnings()

    def test_root_ld_library_path_braces(self):
        self.config_data["environment"] = {"LD_LIBRARY_PATH": "/foo:${LD_LIBRARY_PATH}"}

        self.assert_warnings()

    def test_root_ld_library_path_with_colon_at_start(self):
        self.config_data["environment"] = {"LD_LIBRARY_PATH": ":/foo:/bar"}

        self.assert_warnings()

    def test_root_ld_library_path_with_colon_at_end(self):
        self.config_data["environment"] = {"LD_LIBRARY_PATH": "/foo:/bar:"}

        self.assert_warnings()

    def test_root_ld_library_path_with_colon_at_middle(self):
        self.config_data["environment"] = {"LD_LIBRARY_PATH": "/foo::/bar"}

        self.assert_warnings()

    def test_root_ld_library_path_good(self):
        self.config_data["environment"] = {"LD_LIBRARY_PATH": "/foo:/bar"}

        self.assert_no_warnings()


class TestAppsEnvironmentLibraryPathWarnings(CreateBaseTestCase):
    def setUp(self):
        super().setUp()

        self.config_data["grade"] = "stable"
        self.config_data["apps"] = {
            "app1": {"command": "foo"},
            "app2": {"command": "bar"},
        }

        _create_file(os.path.join(self.prime_dir, "foo"), executable=True)
        _create_file(os.path.join(self.prime_dir, "bar"), executable=True)

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

    def assert_warnings_both(self):
        self.generate_meta_yaml()

        self.assertThat(
            self.fake_logger.output,
            Contains(
                "CVE-2020-27348: A potentially empty LD_LIBRARY_PATH has been set for environment "
                "in 'app1' and 'app2'. The current working directory will be added to the library "
                "path if empty. "
                "This can cause unexpected libraries to be loaded."
            ),
        )

    def assert_warnings_app1(self):
        self.generate_meta_yaml()

        self.assertThat(
            self.fake_logger.output,
            Contains(
                "CVE-2020-27348: A potentially empty LD_LIBRARY_PATH has been set for environment "
                "in 'app1'. The current working directory will be added to the library "
                "path if empty. "
                "This can cause unexpected libraries to be loaded."
            ),
        )

    def assert_no_warnings(self):
        self.generate_meta_yaml()

        self.assertThat(
            self.fake_logger.output,
            Not(
                Contains(
                    "CVE-2020-27348: A potentially empty LD_LIBRARY_PATH has been set for environment "
                    "in 'app1' and 'app2'. The current working directory will be added to the library "
                    "path if empty. "
                    "This can cause unexpected libraries to be loaded."
                )
            ),
        )

    def test_app_ld_library_path_app1(self):
        self.config_data["apps"]["app1"]["environment"] = {
            "LD_LIBRARY_PATH": "/foo:$LD_LIBRARY_PATH"
        }

        self.assert_warnings_app1()

    def test_app_ld_library_path_app1_braces(self):
        self.config_data["apps"]["app1"]["environment"] = {
            "LD_LIBRARY_PATH": "/foo:${LD_LIBRARY_PATH}"
        }

        self.assert_warnings_app1()

    def test_app_ld_library_path_app1_app2(self):
        self.config_data["apps"]["app1"]["environment"] = {
            "LD_LIBRARY_PATH": "/foo:$LD_LIBRARY_PATH"
        }
        self.config_data["apps"]["app2"]["environment"] = {
            "LD_LIBRARY_PATH": "/foo:$LD_LIBRARY_PATH"
        }

        self.assert_warnings_both()

    def test_root_ld_library_path_set(self):
        self.config_data["environment"] = {"LD_LIBRARY_PATH": "/foo:/bar"}

        self.config_data["apps"]["app1"]["environment"] = {
            "LD_LIBRARY_PATH": "/foo:$LD_LIBRARY_PATH"
        }
        self.config_data["apps"]["app2"]["environment"] = {
            "LD_LIBRARY_PATH": "/foo:$LD_LIBRARY_PATH"
        }

        self.assert_no_warnings()

    def test_app_ld_library_path_colon_middle_app1_app2(self):
        self.config_data["apps"]["app1"]["environment"] = {
            "LD_LIBRARY_PATH": "/foo::/bar"
        }
        self.config_data["apps"]["app2"]["environment"] = {
            "LD_LIBRARY_PATH": "/foo::/bar"
        }

        self.assert_warnings_both()


class TestGrade(CreateBaseTestCase):
    def test_stable(self):
        self.config_data["grade"] = "stable"

        self.assertThat(self.generate_meta_yaml()["grade"], Equals("stable"))

    def test_devel(self):
        self.config_data["grade"] = "devel"

        self.assertThat(self.generate_meta_yaml()["grade"], Equals("devel"))

    def test_no_grade(self):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        # Ensure grade defaults to strict if not specified. Also
        # verify that a warning is printed
        self.assertThat(self.generate_meta_yaml()["grade"], Equals("stable"))
        self.assertThat(
            fake_logger.output,
            Contains("'grade' property not specified: defaulting to 'stable'"),
        )


class RequiredGradeTest(CreateBaseTestCase):
    def test_defaults_from_schema(self):
        self.assertThat(self.generate_meta_yaml()["grade"], Equals("stable"))

    def test_stable_required(self):
        global_state_path = "global_state"
        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.project.Project._get_global_state_file_path",
                return_value=global_state_path,
            )
        )

        global_state = states.GlobalState()
        global_state.set_required_grade("stable")
        global_state.save(filepath=global_state_path)

        self.config_data["grade"] = "stable"

        self.assertThat(self.generate_meta_yaml()["grade"], Equals("stable"))

    def test_stable_but_devel_required(self):
        global_state_path = "global_state"
        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.project.Project._get_global_state_file_path",
                return_value=global_state_path,
            )
        )

        global_state = states.GlobalState()
        global_state.set_required_grade("devel")
        global_state.save(filepath=global_state_path)

        self.config_data["grade"] = "stable"

        self.assertRaises(meta_errors.GradeDevelRequiredError, self.generate_meta_yaml)


class CommonIdTestCase(CreateBaseTestCase):
    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        self.create_metadata_file("part", "1.metainfo.xml", "test.id.1")
        self.create_metadata_file("part", "2.metainfo.xml", "test.id.2")

    def create_metadata_file(self, part, name, common_id):
        # Create metadata files
        filename = os.path.join("parts", part, "src", name)
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, "w") as f:
            f.write(
                textwrap.dedent(
                    """\
                    <?xml version="1.0" encoding="utf-8"?>
                    <component type="desktop">
                      <id>{}</id>
                    </component>
                    """.format(
                        common_id
                    )
                )
            )

    def make_snapcraft_project(self, common_id):
        yaml = textwrap.dedent(
            """\
           name: test
           base: core18
           version: "1.0"
           summary: test
           description: test
           confinement: strict
           grade: stable
           adopt-info: part
           apps:
             test-app:
               command: echo
               common-id: {common_id}
           parts:
             part:
               plugin: nil
               parse-info: ["1.metainfo.xml", "2.metainfo.xml"]
           """
        )

        yaml_path = self.make_snapcraft_yaml(yaml.format(common_id=common_id))
        project = Project(snapcraft_yaml_file_path=yaml_path)
        return project_loader.load_config(project)

    def test_common_id(self):
        config = self.make_snapcraft_project(common_id="test.id.2")
        for part in config.parts.all_parts:
            part.makedirs()
            part.mark_pull_done()
            part.build()
            part.stage()
            part.prime()
        _snap_packaging.create_snap_packaging(config)
        self.assertNotIn(
            "is not used in any appstream metafile.", self.fake_logger.output
        )

    def test_common_id_mismatch(self):
        config = self.make_snapcraft_project(common_id="test.id.mismatch")
        for part in config.parts.all_parts:
            part.makedirs()
            part.mark_pull_done()
            part.build()
            part.stage()
            part.prime()
        _snap_packaging.create_snap_packaging(config)
        self.assertIn(
            "Common ID 'test.id.mismatch' specified in app 'test-app' is not used in any metadata file.",
            self.fake_logger.output.strip(),
        )


def _create_file(path, *, content="", executable=False):
    basepath = os.path.dirname(path)
    if basepath:
        os.makedirs(basepath, exist_ok=True)
    mode = "wb" if type(content) == bytes else "w"
    with open(path, mode) as f:
        f.write(content)
    if executable:
        os.chmod(path, 0o755)
