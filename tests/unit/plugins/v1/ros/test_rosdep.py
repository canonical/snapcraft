# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2017-2020 Canonical Ltd
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
import pathlib
import subprocess
from unittest import mock

from testtools.matchers import Equals

import snapcraft
from snapcraft.plugins.v1._ros import rosdep
from tests import unit


class RosdepTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.project = snapcraft.ProjectOptions()

        self.rosdep = rosdep.Rosdep(
            ros_distro="kinetic",
            ros_version="1",
            ros_package_path="package_path",
            rosdep_path="rosdep_path",
            ubuntu_distro="xenial",
            base="core",
            target_arch=self.project._get_stage_packages_target_arch(),
        )

        patcher = mock.patch("snapcraft.repo.Ubuntu")
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.check_output")
        self.check_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_setup(self):
        # Return something other than a Mock to ease later assertions
        self.check_output_mock.return_value = b""

        self.rosdep.setup()

        # Verify that only rosdep was installed (no other .debs)
        self.assertThat(
            self.ubuntu_mock.fetch_stage_packages.mock_calls,
            Equals(
                [
                    mock.call(
                        stage_packages_path=self.rosdep._rosdep_stage_packages_path,
                        package_names=["python-rosdep"],
                        base="core",
                        target_arch=self.project._get_stage_packages_target_arch(),
                    )
                ]
            ),
        )
        self.assertThat(
            self.ubuntu_mock.unpack_stage_packages.mock_calls,
            Equals(
                [
                    mock.call(
                        stage_packages_path=self.rosdep._rosdep_stage_packages_path,
                        install_path=pathlib.Path("rosdep_path/install"),
                    )
                ]
            ),
        )

        # Verify that rosdep was initialized and updated
        self.assertThat(self.check_output_mock.call_count, Equals(2))
        self.check_output_mock.assert_has_calls(
            [
                mock.call(["rosdep", "init"], env=mock.ANY),
                mock.call(["rosdep", "update", "--include-eol-distros"], env=mock.ANY),
            ]
        )

    def test_setup_can_run_multiple_times(self):
        self.rosdep.setup()

        # Make sure running setup() again doesn't have problems with the old
        # environment
        # An exception will be raised if setup can't be called twice.
        self.rosdep.setup()

    def test_setup_initialization_failure(self):
        def run(args, **kwargs):
            if args == ["rosdep", "init"]:
                raise subprocess.CalledProcessError(1, "foo", b"bar")

        self.check_output_mock.side_effect = run

        raised = self.assertRaises(rosdep.RosdepInitializationError, self.rosdep.setup)

        self.assertThat(
            str(raised),
            Equals(
                "Failed to initialize rosdep: Error initializing rosdep database:\nbar"
            ),
        )

    def test_setup_update_failure(self):
        def run(args, **kwargs):
            if args == ["rosdep", "update", "--include-eol-distros"]:
                raise subprocess.CalledProcessError(1, "foo", b"bar")

            return mock.DEFAULT

        self.check_output_mock.side_effect = run

        raised = self.assertRaises(rosdep.RosdepInitializationError, self.rosdep.setup)

        self.assertThat(
            str(raised),
            Equals("Failed to initialize rosdep: Error updating rosdep database:\nbar"),
        )

    def test_get_dependencies(self):
        self.check_output_mock.return_value = b"foo\nbar\nbaz"

        self.assertThat(
            self.rosdep.get_dependencies("foo"), Equals({"foo", "bar", "baz"})
        )

        self.check_output_mock.assert_called_with(
            ["rosdep", "keys", "foo"], env=mock.ANY
        )

    def test_get_dependencies_no_dependencies(self):
        self.check_output_mock.return_value = b""

        self.assertThat(self.rosdep.get_dependencies("foo"), Equals(set()))

    def test_get_dependencies_invalid_package(self):
        self.check_output_mock.side_effect = subprocess.CalledProcessError(1, "foo")

        raised = self.assertRaises(
            rosdep.RosdepPackageNotFoundError, self.rosdep.get_dependencies, "bar"
        )

        self.assertThat(str(raised), Equals("rosdep cannot find Catkin package 'bar'"))

    def test_get_dependencies_entire_workspace(self):
        self.check_output_mock.return_value = b"foo\nbar\nbaz"

        self.assertThat(self.rosdep.get_dependencies(), Equals({"foo", "bar", "baz"}))

        self.check_output_mock.assert_called_with(
            ["rosdep", "keys", "-a", "-i"], env=mock.ANY
        )

    def test_resolve_dependency(self):
        self.check_output_mock.return_value = b"#apt\nmylib-dev"

        self.assertThat(
            self.rosdep.resolve_dependency("foo"), Equals({"apt": {"mylib-dev"}})
        )

        self.check_output_mock.assert_called_with(
            [
                "rosdep",
                "resolve",
                "foo",
                "--rosdistro",
                "kinetic",
                "--os",
                "ubuntu:xenial",
            ],
            env=mock.ANY,
        )

    def test_resolve_invalid_dependency(self):
        self.check_output_mock.side_effect = subprocess.CalledProcessError(1, "foo")

        raised = self.assertRaises(
            rosdep.RosdepDependencyNotResolvedError,
            self.rosdep.resolve_dependency,
            "bar",
        )

        self.assertThat(
            str(raised), Equals("rosdep cannot resolve 'bar' into a valid dependency")
        )

    def test_resolve_unexpected_dependency(self):
        # Note the lack of dependency type here
        self.check_output_mock.return_value = b"mylib-dev"

        raised = self.assertRaises(
            rosdep.RosdepUnexpectedResultError, self.rosdep.resolve_dependency, "bar"
        )

        self.assertThat(
            str(raised),
            Equals(
                "Received unexpected result from rosdep when trying to "
                "resolve 'bar':\nmylib-dev"
            ),
        )

    def test_resolve_no_dependency(self):
        self.check_output_mock.return_value = b"#apt"

        self.assertThat(self.rosdep.resolve_dependency("bar"), Equals({"apt": set()}))

    def test_resolve_multiple_dependencies(self):
        self.check_output_mock.return_value = b"#apt\nlib1 lib2"

        self.assertThat(
            self.rosdep.resolve_dependency("foo"), Equals({"apt": {"lib1", "lib2"}})
        )

    def test_resolve_multiple_dependency_types(self):
        self.check_output_mock.return_value = b"#apt\nlib1\n\n#pip\nlib2"

        self.assertThat(
            self.rosdep.resolve_dependency("foo"),
            Equals({"apt": {"lib1"}, "pip": {"lib2"}}),
        )

    def test_run(self):
        rosdep = self.rosdep
        rosdep._run(["qux"])

        class check_env:
            def __eq__(self, env):
                rosdep_sources_path = rosdep._rosdep_sources_path
                return (
                    env["PATH"]
                    == os.path.join(rosdep._rosdep_install_path, "usr", "bin")
                    and env["PYTHONPATH"]
                    == os.path.join(
                        rosdep._rosdep_install_path,
                        "usr",
                        "lib",
                        "python2.7",
                        "dist-packages",
                    )
                    and env["ROSDEP_SOURCE_PATH"] == rosdep_sources_path
                    and env["ROS_HOME"] == rosdep._rosdep_cache_path
                    and env["ROS_PACKAGE_PATH"] == rosdep._ros_package_path
                    and env["ROS_VERSION"] == rosdep._ros_version
                )

        self.check_output_mock.assert_called_with(mock.ANY, env=check_env())
