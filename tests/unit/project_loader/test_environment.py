# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
import subprocess
import sys
import tempfile
from textwrap import dedent
from unittest import mock

import fixtures
from testtools.matchers import Contains, Equals, Not, StartsWith

import snapcraft
from snapcraft.internal import common
from . import ProjectLoaderBaseTest
from tests.fixture_setup.os_release import FakeOsRelease


class EnvironmentTest(ProjectLoaderBaseTest):
    def setUp(self):
        super().setUp()

        self.snapcraft_yaml = dedent(
            """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                plugin: nil
        """
        )

    def test_config_snap_environment(self):
        project_config = self.make_snapcraft_project(self.snapcraft_yaml)

        lib_paths = [
            os.path.join(self.prime_dir, "lib"),
            os.path.join(self.prime_dir, "usr", "lib"),
            os.path.join(self.prime_dir, "lib", project_config.project.arch_triplet),
            os.path.join(
                self.prime_dir, "usr", "lib", project_config.project.arch_triplet
            ),
        ]
        for lib_path in lib_paths:
            os.makedirs(lib_path)

        environment = project_config.snap_env()
        self.assertThat(
            environment,
            Contains(
                'PATH="{0}/usr/sbin:{0}/usr/bin:{0}/sbin:{0}/bin:$PATH"'.format(
                    self.prime_dir
                )
            ),
        )

        # Ensure that LD_LIBRARY_PATH is present and it contains only the
        # basics.
        paths = []
        for variable in environment:
            if "LD_LIBRARY_PATH" in variable:
                these_paths = variable.split("=")[1].strip()
                paths.extend(these_paths.replace('"', "").split(":"))

        self.assertTrue(len(paths) > 0, "Expected LD_LIBRARY_PATH to be in environment")

        expected = (
            os.path.join(self.prime_dir, i)
            for i in [
                "lib",
                os.path.join("usr", "lib"),
                os.path.join("lib", project_config.project.arch_triplet),
                os.path.join("usr", "lib", project_config.project.arch_triplet),
            ]
        )
        for item in expected:
            self.assertTrue(
                item in paths,
                "Expected LD_LIBRARY_PATH in {!r} to include {!r}".format(paths, item),
            )

    def test_config_snap_environment_with_no_library_paths(self):
        project_config = self.make_snapcraft_project(self.snapcraft_yaml)

        environment = project_config.snap_env()
        self.assertTrue(
            'PATH="{0}/usr/sbin:{0}/usr/bin:{0}/sbin:{0}/bin:$PATH"'.format(
                self.prime_dir
            )
            in environment,
            "Current PATH is {!r}".format(environment),
        )
        for e in environment:
            self.assertFalse(
                "LD_LIBRARY_PATH" in e, "Current environment is {!r}".format(e)
            )

    @mock.patch.object(
        snapcraft.internal.pluginhandler.PluginHandler, "get_primed_dependency_paths"
    )
    def test_config_snap_environment_with_dependencies(self, mock_get_dependencies):
        library_paths = {
            os.path.join(self.prime_dir, "lib1"),
            os.path.join(self.prime_dir, "lib2"),
        }
        mock_get_dependencies.return_value = library_paths
        project_config = self.make_snapcraft_project(self.snapcraft_yaml)

        for lib_path in library_paths:
            os.makedirs(lib_path)

        # Ensure that LD_LIBRARY_PATH is present and it contains the
        # extra dependency paths.
        paths = []
        for variable in project_config.snap_env():
            if "LD_LIBRARY_PATH" in variable:
                these_paths = variable.split("=")[1].strip()
                paths.extend(these_paths.replace('"', "").split(":"))

        self.assertTrue(len(paths) > 0, "Expected LD_LIBRARY_PATH to be in environment")

        expected = (os.path.join(self.prime_dir, i) for i in ["lib1", "lib2"])
        for item in expected:
            self.assertTrue(
                item in paths,
                "Expected LD_LIBRARY_PATH ({!r}) to include {!r}".format(paths, item),
            )

    @mock.patch.object(
        snapcraft.internal.pluginhandler.PluginHandler, "get_primed_dependency_paths"
    )
    def test_config_snap_environment_with_dependencies_but_no_paths(
        self, mock_get_dependencies
    ):
        library_paths = {
            os.path.join(self.prime_dir, "lib1"),
            os.path.join(self.prime_dir, "lib2"),
        }
        mock_get_dependencies.return_value = library_paths
        project_config = self.make_snapcraft_project(self.snapcraft_yaml)

        # Ensure that LD_LIBRARY_PATH is present, but is completey empty since
        # no library paths actually exist.
        for variable in project_config.snap_env():
            self.assertFalse(
                "LD_LIBRARY_PATH" in variable,
                "Expected no LD_LIBRARY_PATH (got {!r})".format(variable),
            )

    def test_config_runtime_environment_ld(self):
        # Place a few ld.so.conf files in supported locations. We expect the
        # contents of these to make it into the LD_LIBRARY_PATH.
        mesa_dir = os.path.join(self.prime_dir, "usr", "lib", "my_arch", "mesa")
        os.makedirs(mesa_dir)
        with open(os.path.join(mesa_dir, "ld.so.conf"), "w") as f:
            f.write("/mesa")

        mesa_egl_dir = os.path.join(self.prime_dir, "usr", "lib", "my_arch", "mesa-egl")
        os.makedirs(mesa_egl_dir)
        with open(os.path.join(mesa_egl_dir, "ld.so.conf"), "w") as f:
            f.write("# Standalone comment\n")
            f.write("/mesa-egl")

        project_config = self.make_snapcraft_project(self.snapcraft_yaml)
        environment = project_config.snap_env()

        # Ensure that the LD_LIBRARY_PATH includes all the above paths
        paths = []
        for variable in environment:
            if "LD_LIBRARY_PATH" in variable:
                these_paths = variable.split("=")[1].strip()
                paths.extend(these_paths.replace('"', "").split(":"))

        self.assertTrue(len(paths) > 0, "Expected LD_LIBRARY_PATH to be in environment")

        expected = (os.path.join(self.prime_dir, i) for i in ["mesa", "mesa-egl"])
        for item in expected:
            self.assertTrue(
                item in paths, 'Expected LD_LIBRARY_PATH to include "{}"'.format(item)
            )

    def test_config_env_dedup(self):
        """Regression test for LP: #1767625.
        Verify that the use of after with multiple parts does not produce
        duplicate exports.
        """
        snapcraft_yaml = dedent(
            """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              main:
                plugin: nil
                after: [part1, part2, part3]
              part1:
                plugin: nil
              part2:
                plugin: nil
              part3:
                plugin: nil
        """
        )
        project_config = self.make_snapcraft_project(snapcraft_yaml)
        part = project_config.parts.get_part("main")
        environment = project_config.parts.build_env_for_part(part, root_part=True)
        # We sort here for equality checking but they should not be sorted
        # for a real case scenario.
        environment.sort()
        self.assertThat(
            environment,
            Equals(
                [
                    (
                        'PATH="{0}/parts/main/install/usr/sbin:'
                        "{0}/parts/main/install/usr/bin:"
                        "{0}/parts/main/install/sbin:"
                        '{0}/parts/main/install/bin:$PATH"'
                    ).format(self.path),
                    (
                        'PATH="{0}/stage/usr/sbin:'
                        "{0}/stage/usr/bin:"
                        "{0}/stage/sbin:"
                        '{0}/stage/bin:$PATH"'
                    ).format(self.path),
                    'PERL5LIB="{0}/stage/usr/share/perl5/"'.format(self.path),
                    'SNAPCRAFT_ARCH_TRIPLET="{}"'.format(
                        project_config.project.arch_triplet
                    ),
                    'SNAPCRAFT_EXTENSIONS_DIR="{}"'.format(common.get_extensionsdir()),
                    'SNAPCRAFT_PARALLEL_BUILD_COUNT="2"',
                    'SNAPCRAFT_PART_BUILD="{}/parts/main/build"'.format(self.path),
                    'SNAPCRAFT_PART_INSTALL="{}/parts/main/install"'.format(self.path),
                    'SNAPCRAFT_PART_SRC="{}/parts/main/src"'.format(self.path),
                    'SNAPCRAFT_PRIME="{}/prime"'.format(self.path),
                    'SNAPCRAFT_PROJECT_GRADE="stable"',
                    'SNAPCRAFT_PROJECT_NAME="test"',
                    'SNAPCRAFT_PROJECT_VERSION="1"',
                    'SNAPCRAFT_STAGE="{}/stage"'.format(self.path),
                ]
            ),
        )

    def test_config_stage_environment_confinement_classic(self):
        self.useFixture(FakeOsRelease())

        snapcraft_yaml = dedent(
            """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: classic
            grade: stable
            base: core

            parts:
              part1:
                plugin: nil
        """
        )
        project_config = self.make_snapcraft_project(snapcraft_yaml)
        part = project_config.parts.get_part("part1")
        environment = project_config.parts.build_env_for_part(part, root_part=True)
        self.assertIn(
            'LD_LIBRARY_PATH="$LD_LIBRARY_PATH:{base_core_path}/lib:'
            "{base_core_path}/usr/lib:{base_core_path}/lib/{arch_triplet}:"
            '{base_core_path}/usr/lib/{arch_triplet}"'.format(
                base_core_path=self.base_environment.core_path,
                arch_triplet=project_config.project.arch_triplet,
            ),
            environment,
        )

    def test_stage_environment_confinement_classic_with_incompat_host(self):
        self.useFixture(FakeOsRelease(version_codename="incompatible-fake"))

        snapcraft_yaml = dedent(
            """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: classic
            grade: stable
            base: core

            parts:
              part1:
                plugin: nil
        """
        )
        project_config = self.make_snapcraft_project(snapcraft_yaml)
        part = project_config.parts.get_part("part1")
        environment = project_config.parts.build_env_for_part(part, root_part=True)
        for env_item in environment:
            self.assertThat(env_item, Not(StartsWith("LD_LIBRARY_PATH")))

    def test_stage_environment_confinement_classic_with_incompat_base(self):
        snapcraft_yaml = """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: classic
            grade: stable
            base: fake-core

            parts:
              part1:
                plugin: nil
        """
        project_config = self.make_snapcraft_project(snapcraft_yaml)
        part = project_config.parts.get_part("part1")
        environment = project_config.parts.build_env_for_part(part, root_part=True)
        for env_item in environment:
            self.assertThat(env_item, Not(StartsWith("LD_LIBRARY_PATH")))

    def test_config_stage_environment(self):
        arch_triplet = snapcraft.ProjectOptions().arch_triplet
        paths = [
            os.path.join(self.stage_dir, "lib"),
            os.path.join(self.stage_dir, "lib", arch_triplet),
            os.path.join(self.stage_dir, "usr", "lib"),
            os.path.join(self.stage_dir, "usr", "lib", arch_triplet),
            os.path.join(self.stage_dir, "include"),
            os.path.join(self.stage_dir, "usr", "include"),
            os.path.join(self.stage_dir, "include", arch_triplet),
            os.path.join(self.stage_dir, "usr", "include", arch_triplet),
        ]
        for path in paths:
            os.makedirs(path)

        project_config = self.make_snapcraft_project(self.snapcraft_yaml)
        environment = project_config.stage_env()

        self.assertTrue(
            'PATH="{0}/usr/sbin:{0}/usr/bin:{0}/sbin:{0}/bin:$PATH"'.format(
                self.stage_dir
            )
            in environment
        )
        self.assertTrue(
            'LD_LIBRARY_PATH="$LD_LIBRARY_PATH:{stage_dir}/lib:'
            "{stage_dir}/usr/lib:{stage_dir}/lib/{arch_triplet}:"
            '{stage_dir}/usr/lib/{arch_triplet}"'.format(
                stage_dir=self.stage_dir,
                arch_triplet=project_config.project.arch_triplet,
            )
            in environment,
            "Current environment is {!r}".format(environment),
        )
        self.assertTrue(
            'CFLAGS="$CFLAGS -I{stage_dir}/include -I{stage_dir}/usr/include '
            "-I{stage_dir}/include/{arch_triplet} "
            '-I{stage_dir}/usr/include/{arch_triplet}"'.format(
                stage_dir=self.stage_dir,
                arch_triplet=project_config.project.arch_triplet,
            )
            in environment,
            "Current environment is {!r}".format(environment),
        )
        self.assertTrue(
            'CPPFLAGS="$CPPFLAGS -I{stage_dir}/include '
            "-I{stage_dir}/usr/include "
            "-I{stage_dir}/include/{arch_triplet} "
            '-I{stage_dir}/usr/include/{arch_triplet}"'.format(
                stage_dir=self.stage_dir,
                arch_triplet=project_config.project.arch_triplet,
            )
            in environment,
            "Current environment is {!r}".format(environment),
        )
        self.assertTrue(
            'CXXFLAGS="$CXXFLAGS -I{stage_dir}/include '
            "-I{stage_dir}/usr/include "
            "-I{stage_dir}/include/{arch_triplet} "
            '-I{stage_dir}/usr/include/{arch_triplet}"'.format(
                stage_dir=self.stage_dir,
                arch_triplet=project_config.project.arch_triplet,
            )
            in environment,
            "Current environment is {!r}".format(environment),
        )
        self.assertTrue(
            'LDFLAGS="$LDFLAGS -L{stage_dir}/lib -L{stage_dir}/usr/lib '
            "-L{stage_dir}/lib/{arch_triplet} "
            '-L{stage_dir}/usr/lib/{arch_triplet}"'.format(
                stage_dir=self.stage_dir,
                arch_triplet=project_config.project.arch_triplet,
            )
            in environment,
            "Current environment is {!r}".format(environment),
        )
        self.assertTrue(
            'PERL5LIB="{}/usr/share/perl5/"'.format(self.stage_dir) in environment
        )

    def test_parts_build_env_ordering_with_deps(self):
        snapcraft_yaml = dedent(
            """\
            name: test
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                plugin: nil
              part2:
                plugin: nil
                after: [part1]
        """
        )

        self.useFixture(fixtures.EnvironmentVariable("PATH", "/bin"))

        arch_triplet = snapcraft.ProjectOptions().arch_triplet
        self.maxDiff = None
        paths = [
            os.path.join(self.stage_dir, "lib"),
            os.path.join(self.stage_dir, "lib", arch_triplet),
            os.path.join(self.stage_dir, "usr", "lib"),
            os.path.join(self.stage_dir, "usr", "lib", arch_triplet),
            os.path.join(self.stage_dir, "include"),
            os.path.join(self.stage_dir, "usr", "include"),
            os.path.join(self.stage_dir, "include", arch_triplet),
            os.path.join(self.stage_dir, "usr", "include", arch_triplet),
            os.path.join(self.parts_dir, "part1", "install", "include"),
            os.path.join(self.parts_dir, "part1", "install", "lib"),
            os.path.join(self.parts_dir, "part2", "install", "include"),
            os.path.join(self.parts_dir, "part2", "install", "lib"),
        ]
        for path in paths:
            os.makedirs(path)

        project_config = self.make_snapcraft_project(snapcraft_yaml)
        part2 = [
            part for part in project_config.parts.all_parts if part.name == "part2"
        ][0]
        env = project_config.parts.build_env_for_part(part2)
        env_lines = "\n".join(["export {}\n".format(e) for e in env])

        shell_env = {
            "CFLAGS": "-I/user-provided",
            "CXXFLAGS": "-I/user-provided",
            "CPPFLAGS": "-I/user-provided",
            "LDFLAGS": "-L/user-provided",
            "LD_LIBRARY_PATH": "/user-provided",
        }

        def get_envvar(envvar):
            with tempfile.NamedTemporaryFile(mode="w+") as f:
                f.write(env_lines)
                f.write("echo ${}".format(envvar))
                f.flush()
                output = subprocess.check_output(["/bin/sh", f.name], env=shell_env)
            return output.decode(sys.getfilesystemencoding()).strip()

        expected_cflags = (
            "-I/user-provided "
            "-I{parts_dir}/part2/install/include -I{stage_dir}/include "
            "-I{stage_dir}/usr/include "
            "-I{stage_dir}/include/{arch_triplet} "
            "-I{stage_dir}/usr/include/{arch_triplet}".format(
                parts_dir=self.parts_dir,
                stage_dir=self.stage_dir,
                arch_triplet=project_config.project.arch_triplet,
            )
        )
        self.assertThat(get_envvar("CFLAGS"), Equals(expected_cflags))
        self.assertThat(get_envvar("CXXFLAGS"), Equals(expected_cflags))
        self.assertThat(get_envvar("CPPFLAGS"), Equals(expected_cflags))

        self.assertThat(
            get_envvar("LDFLAGS"),
            Equals(
                "-L/user-provided "
                "-L{parts_dir}/part2/install/lib -L{stage_dir}/lib "
                "-L{stage_dir}/usr/lib -L{stage_dir}/lib/{arch_triplet} "
                "-L{stage_dir}/usr/lib/{arch_triplet}".format(
                    parts_dir=self.parts_dir,
                    stage_dir=self.stage_dir,
                    arch_triplet=project_config.project.arch_triplet,
                )
            ),
        )

        self.assertThat(
            get_envvar("LD_LIBRARY_PATH"),
            Equals(
                "/user-provided:"
                "{parts_dir}/part2/install/lib:"
                "{stage_dir}/lib:"
                "{stage_dir}/usr/lib:"
                "{stage_dir}/lib/{arch_triplet}:"
                "{stage_dir}/usr/lib/{arch_triplet}".format(
                    parts_dir=self.parts_dir,
                    stage_dir=self.stage_dir,
                    arch_triplet=project_config.project.arch_triplet,
                )
            ),
        )

    @mock.patch("multiprocessing.cpu_count", return_value=42)
    def test_parts_build_env_contains_parallel_build_count(self, cpu_mock):
        project_config = self.make_snapcraft_project(self.snapcraft_yaml)
        part1 = [
            part for part in project_config.parts.all_parts if part.name == "part1"
        ][0]
        env = project_config.parts.build_env_for_part(part1)
        self.assertThat(env, Contains('SNAPCRAFT_PARALLEL_BUILD_COUNT="42"'))

    def test_extension_dir(self):
        common.set_extensionsdir("/foo")
        project_config = self.make_snapcraft_project(self.snapcraft_yaml)
        part1 = [
            part for part in project_config.parts.all_parts if part.name == "part1"
        ][0]
        env = project_config.parts.build_env_for_part(part1)
        self.assertThat(env, Contains('SNAPCRAFT_EXTENSIONS_DIR="/foo"'))
