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

import json
import logging
import os
from subprocess import CalledProcessError
from unittest.mock import call, patch, ANY

import fixtures
from testtools.matchers import Equals

from snapcraft.project import Project
from snapcraft.project._project_options import _get_deb_arch
from snapcraft.internal import errors, lxd
from tests import fixture_setup, unit


class LXDBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.fake_lxd = fixture_setup.FakeLXD()
        self.useFixture(self.fake_lxd)
        self.fake_lxd.kernel_arch = self.server

        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        patcher = patch("snapcraft.internal.lxd._containerbuild.SnapInjector")
        self.snap_injector_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        self.useFixture(self.snapcraft_yaml)
        self.project = Project(
            snapcraft_yaml_file_path=self.snapcraft_yaml.snapcraft_yaml_file_path,
            target_deb_arch=self.target_arch,
        )

        self.useFixture(fixtures.EnvironmentVariable("SUDO_UID", "1000"))


class LXDTestCase(LXDBaseTestCase):

    scenarios = [
        ("local", dict(remote="local", target_arch=None, server="x86_64")),
        ("remote", dict(remote="myremote", target_arch=None, server="x86_64")),
        (
            "cross",
            dict(remote="local", target_arch="armhf", server="x86_64", cross=True),
        ),
        ("arm remote", dict(remote="pi", target_arch=None, server="armv7l")),
        ("arm same", dict(remote="pi", target_arch="armhf", server="armv7l")),
        (
            "arm cross",
            dict(remote="pi", target_arch="arm64", server="armv7l", cross=True),
        ),
    ]


class CleanbuilderTestCase(LXDTestCase):
    def make_containerbuild(self):
        return lxd.Cleanbuilder(
            output="snap.snap",
            source="project.tar",
            project=self.project,
            remote=self.remote,
        )

    @patch("snapcraft.internal.lxd.Containerbuild._container_run")
    @patch("snapcraft.internal.lxd.Containerbuild._inject_snapcraft")
    @patch("petname.Generate")
    def test_cleanbuild(self, mock_pet, mock_inject, mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd

        mock_pet.return_value = "my-pet"

        project_folder = "/root/build_{}".format(self.project.info.name)
        self.make_containerbuild().execute()

        self.assertIn(
            "Waiting for a network connection...\nNetwork connection established\n",
            self.fake_logger.output,
        )

        args = []
        if self.target_arch:
            self.assertIn(
                "Setting target machine to '{}'\n".format(self.target_arch),
                self.fake_logger.output,
            )
            args += ["--target-arch", self.target_arch]

        container_name = "{}:snapcraft-my-pet".format(self.remote)
        self.fake_lxd.check_call_mock.assert_has_calls(
            [
                call(["lxc", "launch", "-e", "ubuntu:xenial", container_name]),
                call(
                    ["lxc", "config", "set", container_name, "raw.idmap", "both 1000 0"]
                ),
                call(
                    [
                        "lxc",
                        "config",
                        "set",
                        container_name,
                        "environment.SNAPCRAFT_SETUP_CORE",
                        "1",
                    ]
                ),
                call(
                    [
                        "lxc",
                        "config",
                        "set",
                        container_name,
                        "environment.SNAPCRAFT_MANAGED_HOST",
                        "yes",
                    ]
                ),
                call(
                    [
                        "lxc",
                        "config",
                        "set",
                        container_name,
                        "environment.LC_ALL",
                        "C.UTF-8",
                    ]
                ),
                call(
                    [
                        "lxc",
                        "config",
                        "set",
                        container_name,
                        "environment.SNAPCRAFT_IMAGE_INFO",
                        '{"fingerprint": "test-fingerprint", '
                        '"architecture": "test-architecture", '
                        '"created_at": "test-created-at"}',
                    ]
                ),
                call(
                    [
                        "lxc",
                        "file",
                        "push",
                        os.path.realpath("project.tar"),
                        "{}{}/project.tar".format(container_name, project_folder),
                    ]
                ),
                call(
                    [
                        "lxc",
                        "file",
                        "pull",
                        "{}{}/snap.snap".format(container_name, project_folder),
                        "snap.snap",
                    ]
                ),
                call(["lxc", "stop", "-f", container_name]),
            ]
        )
        mock_container_run.assert_has_calls(
            [
                call(["python3", "-c", ANY]),
                call(["cloud-init", "status", "--wait"], hide_output=True),
                call(["apt-get", "update"]),
                call(["mkdir", project_folder]),
                call(["tar", "xvf", "project.tar"], cwd=project_folder),
                call(
                    ["snapcraft", "snap", "--output", "snap.snap", *args],
                    cwd=project_folder,
                    user="root",
                ),
            ]
        )
        # Ensure there's no unexpected calls eg. two network checks
        self.assertThat(mock_container_run.call_count, Equals(6))
        self.fake_lxd.check_call_mock.assert_has_calls(
            [
                call(
                    [
                        "lxc",
                        "file",
                        "pull",
                        "{}{}/snap.snap".format(container_name, project_folder),
                        "snap.snap",
                    ]
                ),
                call(["lxc", "stop", "-f", container_name]),
            ]
        )
        self.fake_lxd.check_output_mock.assert_has_calls(
            [
                call(
                    [
                        "lxc",
                        "image",
                        "list",
                        "--format=json",
                        "ubuntu:xenial/{}".format(_get_deb_arch(self.server)),
                    ]
                )
            ]
        )

    def test_failed_container_never_created(self):
        def call_effect(*args, **kwargs):
            if args[0][:2] == ["lxc", "launch"]:
                raise CalledProcessError(returncode=255, cmd=args[0])
            return self.fake_lxd.check_output_side_effect()(*args, **kwargs)

        self.fake_lxd.check_call_mock.side_effect = call_effect

        self.assertRaises(
            lxd.errors.ContainerCreationFailedError, self.make_containerbuild().execute
        )
        self.assertThat(self.fake_lxd.status, Equals(None))
        # lxc launch should fail and no further commands should come after that


class ContainerbuildTestCase(LXDTestCase):
    def make_containerbuild(self):
        return lxd.Cleanbuilder(
            output="snap.snap",
            source="project.tar",
            project=self.project,
            remote=self.remote,
        )

    def test_parts_uri_set(self):
        self.useFixture(fixtures.EnvironmentVariable("SNAPCRAFT_PARTS_URI", "foo"))
        self.make_containerbuild().execute()
        self.fake_lxd.check_call_mock.assert_has_calls(
            [
                call(
                    [
                        "lxc",
                        "config",
                        "set",
                        self.fake_lxd.name,
                        "environment.SNAPCRAFT_PARTS_URI",
                        "foo",
                    ]
                )
            ]
        )

    def test_build_info_set(self):
        self.useFixture(
            fixtures.EnvironmentVariable(
                "SNAPCRAFT_BUILD_INFO", "test_build_info_value"
            )
        )
        self.make_containerbuild().execute()
        self.fake_lxd.check_call_mock.assert_has_calls(
            [
                call(
                    [
                        "lxc",
                        "config",
                        "set",
                        self.fake_lxd.name,
                        "environment.SNAPCRAFT_BUILD_INFO",
                        "test_build_info_value",
                    ]
                )
            ]
        )

    def test_image_info_merged(self):
        test_image_info = '{"build_url": "test-build-url"}'
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_IMAGE_INFO", test_image_info)
        )
        self.make_containerbuild().execute()
        self.fake_lxd.check_call_mock.assert_has_calls(
            [
                call(
                    [
                        "lxc",
                        "config",
                        "set",
                        self.fake_lxd.name,
                        "environment.SNAPCRAFT_IMAGE_INFO",
                        '{"fingerprint": "test-fingerprint", '
                        '"architecture": "test-architecture", '
                        '"created_at": "test-created-at", '
                        '"build_url": "test-build-url"}',
                    ]
                )
            ]
        )

    def test_image_info_invalid(self):
        test_image_info = "not-json"
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_IMAGE_INFO", test_image_info)
        )
        self.assertRaises(
            errors.InvalidContainerImageInfoError, self.make_containerbuild().execute
        )

    def test_architecture_syntax_raises_error(self):
        def call_effect(*args, **kwargs):
            if args[0][:2] == ["lxc", "info"]:
                return "Architecture:foo".encode("utf-8")
            return self.fake_lxd.check_output_side_effect()(*args, **kwargs)

        self.fake_lxd.check_output_mock.side_effect = call_effect

        self.assertRaises(
            lxd.errors.ContainerArchitectureError,
            self.make_containerbuild()._get_container_arch,
        )

    def test_architecture_missing_raises_error(self):
        def call_effect(*args, **kwargs):
            if args[0][:2] == ["lxc", "info"]:
                return "Status: Stopped".encode("utf-8")
            return self.fake_lxd.check_output_side_effect()(*args, **kwargs)

        self.fake_lxd.check_output_mock.side_effect = call_effect

        self.assertRaises(
            lxd.errors.ContainerArchitectureError,
            self.make_containerbuild()._get_container_arch,
        )

    def test_architecture_unknown_raises_error(self):
        def call_effect(*args, **kwargs):
            if args[0][:2] == ["lxc", "info"]:
                return "Architecture: invalid".encode("utf-8")
            return self.fake_lxd.check_output_side_effect()(*args, **kwargs)

        self.fake_lxd.check_output_mock.side_effect = call_effect

        self.assertRaises(
            lxd.errors.ContainerArchitectureError,
            self.make_containerbuild()._get_container_arch,
        )

    def test_wait_for_network_loops(self):
        self.fake_lxd.check_call_mock.side_effect = CalledProcessError(-1, ["my-cmd"])

        builder = self.make_containerbuild()

        self.assertRaises(lxd.errors.ContainerNetworkError, builder._wait_for_network)

    def test_failed_build_with_debug(self):
        def call_effect(*args, **kwargs):
            if "snapcraft snap --output snap.snap" in " ".join(args[0]):
                raise CalledProcessError(returncode=255, cmd=args[0])
            return self.fake_lxd.check_output_side_effect()(*args, **kwargs)

        self.fake_lxd.check_call_mock.side_effect = call_effect

        self.project = Project(
            snapcraft_yaml_file_path=self.snapcraft_yaml.snapcraft_yaml_file_path,
            debug=True,
        )
        self.make_containerbuild().execute()

        self.fake_lxd.check_call_mock.assert_has_calls(
            [call(["lxc", "exec", self.fake_lxd.name, "--", "bash", "-i"])]
        )

    @patch("snapcraft.internal.lxd.Containerbuild._container_run")
    def test_failed_build_without_debug(self, mock_run):
        call_list = []

        def run_effect(*args, **kwargs):
            call_list.append(args[0])
            if args[0][:4] == ["snapcraft", "snap", "--output", "snap.snap"]:
                raise CalledProcessError(returncode=255, cmd=args[0])

        mock_run.side_effect = run_effect

        self.assertRaises(CalledProcessError, self.make_containerbuild().execute)

        self.assertNotIn(["bash", "-i"], call_list)

    @patch("snapcraft.internal.lxd.Containerbuild._container_run")
    def test_lxc_check_fails(self, mock_run):
        self.fake_lxd.check_output_mock.side_effect = FileNotFoundError("lxc")

        self.assertRaises(
            lxd.errors.ContainerLXDNotInstalledError, self.make_containerbuild().execute
        )

    @patch("snapcraft.internal.lxd.Containerbuild._container_run")
    def test_remote_does_not_exist(self, mock_run):
        self.fake_lxd.check_output_mock.side_effect = CalledProcessError(
            255, ["lxd", "list", self.remote]
        )

        raised = self.assertRaises(
            lxd.errors.ContainerLXDRemoteNotFoundError,
            self.make_containerbuild().execute,
        )
        self.assertThat(raised.remote, Equals(self.remote))


class LocalProjectTestCase(LXDTestCase):
    def make_containerbuild(self):
        return lxd.Project(
            output="snap.snap", source="project.tar", project=self.project
        )

    def test_init_failed(self):
        def call_effect(*args, **kwargs):
            if args[0][:2] == ["lxc", "init"]:
                raise CalledProcessError(returncode=255, cmd=args[0])
            return self.fake_lxd.check_output_side_effect()(*args, **kwargs)

        self.fake_lxd.check_call_mock.side_effect = call_effect

        self.assertRaises(
            lxd.errors.ContainerCreationFailedError, self.make_containerbuild().execute
        )
        self.assertThat(self.fake_lxd.status, Equals(None))
        # lxc launch should fail and no further commands should come after that

    @patch("snapcraft.internal.lxd.Containerbuild._container_run")
    def test_start_failed(self, mock_container_run):
        mock_container_run.side_effect = lambda cmd, **kwargs: cmd

        def call_effect(*args, **kwargs):
            if args[0][:2] == ["lxc", "start"]:
                raise CalledProcessError(returncode=255, cmd=args[0])
            return d(*args, **kwargs)

        d = self.fake_lxd.check_call_mock.side_effect
        self.fake_lxd.check_call_mock.side_effect = call_effect

        self.assertRaises(
            lxd.errors.ContainerStartFailedError, self.make_containerbuild().execute
        )
        # Should not attempt to stop a container that wasn't started
        self.assertNotIn(
            call(["lxc", "stop", "-f", self.fake_lxd.name]),
            self.fake_lxd.check_call_mock.call_args_list,
        )


class FailedImageInfoTestCase(LXDBaseTestCase):

    remote = "local"
    server = "x86_64"
    target_arch = None

    scenarios = [
        (
            "CalledProcessError",
            dict(
                exception=CalledProcessError,
                kwargs=dict(cmd="testcmd", returncode=1, output="test output"),
                expected_warn=(
                    "Failed to get container image info: "
                    "`lxc image list --format=json ubuntu:xenial/amd64` "
                    "returned with exit code 1, output: test output\n"
                    "It will not be recorded in manifest.\n"
                ),
            ),
        ),
        (
            "JSONDecodeError",
            dict(
                exception=json.decoder.JSONDecodeError,
                kwargs=dict(msg="dummy", doc="dummy", pos=1),
                expected_warn=(
                    "Failed to get container image info: Not in JSON format\n"
                    "It will not be recorded in manifest.\n"
                ),
            ),
        ),
    ]

    def make_containerbuild(self):
        return lxd.Project(
            output="snap.snap", source="project.tar", project=self.project
        )

    def test_failed_image_info_just_warns(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.WARN)
        self.useFixture(self.fake_logger)

        def call_effect(*args, **kwargs):
            if args[0][:3] == ["lxc", "image", "list"]:
                raise self.exception(**self.kwargs)
            return d(*args, **kwargs)

        d = self.fake_lxd.check_output_mock.side_effect

        self.fake_lxd.check_output_mock.side_effect = call_effect

        self.make_containerbuild().execute()
        self.assertEqual(self.fake_logger.output, self.expected_warn)


class SnapOutputTestCase(unit.TestCase):

    scenarios = [
        (
            "all info",
            dict(
                name="name",
                version="version",
                architectures=["arm64"],
                expected="name_version_arm64.snap",
            ),
        ),
        (
            "missing version",
            dict(
                name="name",
                version=None,
                architectures=["arm64"],
                expected="name_arm64.snap",
            ),
        ),
    ]

    @patch(
        "snapcraft.internal.lxd._containerbuild._get_default_remote",
        return_value="local",
    )
    def test_output_set_correctly(self, default_remote_mock):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path,
            name=self.name,
            version=self.version,
            architectures=self.architectures,
        )
        self.useFixture(snapcraft_yaml)
        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )

        instance = lxd.Containerbuild(
            project=project, source="tarball.tgz", container_name="name"
        )
        self.assertThat(instance.snap_filename, Equals(self.expected))
