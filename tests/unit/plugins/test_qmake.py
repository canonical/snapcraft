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

from unittest import mock
from testtools.matchers import Equals, HasLength

import snapcraft
from snapcraft.plugins import qmake
from tests import unit


class QMakeTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch("snapcraft.internal.common.run")
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("sys.stdout")
        patcher.start()
        self.addCleanup(patcher.stop)

        class Options:
            options = []
            qt_version = "qt5"
            project_files = []

        self.options = Options()

    def test_schema(self):
        schema = qmake.QmakePlugin.schema()

        # Verify the presence of all properties
        properties = schema["properties"]
        self.assertTrue(
            "options" in properties, 'Expected "options" to be included in properties'
        )
        self.assertTrue(
            "qt-version" in properties,
            'Expected "qt-version" to be included in properties',
        )

        # Check options property
        options = properties["options"]
        for item in ["type", "minitems", "uniqueItems", "items", "default"]:
            self.assertTrue(
                item in options,
                'Expected {!r} to be included in "options"'.format(item),
            )

        options_type = options["type"]
        self.assertThat(
            options_type,
            Equals("array"),
            'Expected "options" "type" to be "array", but it '
            'was "{}"'.format(options_type),
        )

        options_minitems = options["minitems"]
        self.assertThat(
            options_minitems,
            Equals(1),
            'Expected "options" "minitems" to be 1, but '
            "it was {}".format(options_minitems),
        )

        self.assertTrue(options["uniqueItems"])

        options_default = options["default"]
        self.assertThat(
            options_default,
            Equals([]),
            'Expected "options" "default" to be [], but '
            "it was {}".format(options_default),
        )

        options_items = options["items"]
        self.assertTrue(
            "type" in options_items,
            'Expected "type" to be included in "options" ' '"items"',
        )

        options_items_type = options_items["type"]
        self.assertThat(
            options_items_type,
            Equals("string"),
            'Expected "options" "items" "type" to be '
            '"string", but it was "{}"'.format(options_items_type),
        )

        # Check qt-version property
        qt_version = properties["qt-version"]
        self.assertTrue(
            "enum" in qt_version, 'Expected "enum" to be included in "qt_version"'
        )

        qt_version_enum = qt_version["enum"]
        # Using sets for order independence in the comparison
        self.assertThat(set(qt_version_enum), Equals(set(["qt4", "qt5"])))

        qt_version_default = qt_version["default"]
        self.assertThat(
            qt_version_default,
            Equals("qt5"),
            'Expected "qt_version" "default" to be "qt5", but '
            "it was {}".format(qt_version_default),
        )

        self.assertTrue(
            "build-properties" in schema,
            'Expected schema to include "build-properties"',
        )

        self.assertTrue(
            "pull-properties" in schema, 'Expected schema to include "pull-properties"'
        )

        # Check project-files property
        project_files = properties["project-files"]
        for item in ["type", "minitems", "uniqueItems", "items", "default"]:
            self.assertTrue(
                item in options,
                'Expected {!r} to be included in "project_files"'.format(item),
            )

        project_files_type = project_files["type"]
        self.assertThat(
            project_files_type,
            Equals("array"),
            'Expected "project_files" "type" to be "array", but '
            'it was "{}"'.format(project_files_type),
        )

        project_files_minitems = project_files["minitems"]
        self.assertThat(
            project_files_minitems,
            Equals(1),
            'Expected "project_files" "minitems" to be 1, but '
            "it was {}".format(project_files_minitems),
        )

        self.assertTrue(project_files["uniqueItems"])

        project_files_default = project_files["default"]
        self.assertThat(
            project_files_default,
            Equals([]),
            'Expected "project_files" "default" to be [], but '
            "it was {}".format(project_files_default),
        )

        project_files_items = project_files["items"]
        self.assertTrue(
            "type" in project_files_items,
            'Expected "type" to be included in "project_files" ' '"items"',
        )

        project_files_items_type = project_files_items["type"]
        self.assertThat(
            project_files_items_type,
            Equals("string"),
            'Expected "project_files" "items" "type" to be '
            '"string", but it was "{}"'.format(project_files_items_type),
        )

    def test_get_build_properties(self):
        expected_build_properties = ["options", "project-files"]
        resulting_build_properties = qmake.QmakePlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    def test_get_pull_properties(self):
        expected_pull_properties = ["qt-version"]
        resulting_pull_properties = qmake.QmakePlugin.get_pull_properties()

        self.assertThat(
            resulting_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_unsupported_qt_version(self):
        self.options.qt_version = "qt3"
        raised = self.assertRaises(
            RuntimeError,
            qmake.QmakePlugin,
            "test-part",
            self.options,
            self.project_options,
        )

        self.assertThat(str(raised), Equals("Unsupported Qt version: 'qt3'"))

    def test_build_referencing_sourcedir_if_no_subdir(self):
        plugin = qmake.QmakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.build()

        self.run_mock.assert_has_calls(
            [
                mock.call(["qmake"], cwd=plugin.builddir, env=mock.ANY),
                mock.call(["make", "-j2"], cwd=plugin.builddir, env=mock.ANY),
                mock.call(
                    ["make", "install", "INSTALL_ROOT={}".format(plugin.installdir)],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
            ]
        )

    def test_build_referencing_sourcedir_with_subdir(self):
        self.options.source_subdir = "subdir"

        plugin = qmake.QmakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.build()

        self.run_mock.assert_has_calls(
            [
                mock.call(["qmake"], cwd=plugin.builddir, env=mock.ANY),
                mock.call(["make", "-j2"], cwd=plugin.builddir, env=mock.ANY),
                mock.call(
                    ["make", "install", "INSTALL_ROOT={}".format(plugin.installdir)],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
            ]
        )

    def test_build_with_project_file(self):
        self.options.project_files = ["project_file.pro"]

        plugin = qmake.QmakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.build()

        path_to_pro_file = os.path.join(plugin.sourcedir, "project_file.pro")
        self.run_mock.assert_has_calls(
            [
                mock.call(
                    ["qmake", path_to_pro_file], cwd=plugin.builddir, env=mock.ANY
                ),
                mock.call(["make", "-j2"], cwd=plugin.builddir, env=mock.ANY),
                mock.call(
                    ["make", "install", "INSTALL_ROOT={}".format(plugin.installdir)],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
            ]
        )

    def test_build_with_project_file_with_subdir(self):
        self.options.source_subdir = "subdir"
        self.options.project_files = ["project_file.pro"]

        plugin = qmake.QmakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.build()

        path_to_pro_file = os.path.join(
            plugin.sourcedir, plugin.options.source_subdir, "project_file.pro"
        )
        self.run_mock.assert_has_calls(
            [
                mock.call(
                    ["qmake", path_to_pro_file], cwd=plugin.builddir, env=mock.ANY
                ),
                mock.call(["make", "-j2"], cwd=plugin.builddir, env=mock.ANY),
                mock.call(
                    ["make", "install", "INSTALL_ROOT={}".format(plugin.installdir)],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
            ]
        )

    def test_build_with_options(self):
        self.options.options = ["-foo"]

        plugin = qmake.QmakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.build()

        self.run_mock.assert_has_calls(
            [
                mock.call(["qmake", "-foo"], cwd=plugin.builddir, env=mock.ANY),
                mock.call(["make", "-j2"], cwd=plugin.builddir, env=mock.ANY),
                mock.call(
                    ["make", "install", "INSTALL_ROOT={}".format(plugin.installdir)],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
            ]
        )

    def test_build_with_libs_and_includes_in_installdir(self):
        plugin = qmake.QmakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)
        os.makedirs(os.path.join(plugin.installdir, "include"))
        os.makedirs(os.path.join(plugin.installdir, "lib"))
        plugin.build()

        self.run_mock.assert_has_calls(
            [
                mock.call(
                    [
                        "qmake",
                        'LIBS+="-L{}/lib"'.format(plugin.installdir),
                        'INCLUDEPATH+="{}/include"'.format(plugin.installdir),
                    ],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
                mock.call(["make", "-j2"], cwd=plugin.builddir, env=mock.ANY),
                mock.call(
                    ["make", "install", "INSTALL_ROOT={}".format(plugin.installdir)],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
            ]
        )

    def test_build_with_libs_and_includes_in_stagedir(self):
        plugin = qmake.QmakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)
        os.makedirs(os.path.join(plugin.project.stage_dir, "include"))
        os.makedirs(os.path.join(plugin.project.stage_dir, "lib"))
        plugin.build()

        self.run_mock.assert_has_calls(
            [
                mock.call(
                    [
                        "qmake",
                        'LIBS+="-L{}/lib"'.format(plugin.project.stage_dir),
                        'INCLUDEPATH+="{}/include"'.format(plugin.project.stage_dir),
                    ],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
                mock.call(["make", "-j2"], cwd=plugin.builddir, env=mock.ANY),
                mock.call(
                    ["make", "install", "INSTALL_ROOT={}".format(plugin.installdir)],
                    cwd=plugin.builddir,
                    env=mock.ANY,
                ),
            ]
        )

    def test_build_environment_qt4(self):
        self.options.qt_version = "qt4"
        plugin = qmake.QmakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.build()

        self.assert_expected_environment({"QT_SELECT": "qt4"})

    def test_build_environment_qt5(self):
        self.options.qt_version = "qt5"
        plugin = qmake.QmakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)
        plugin.build()

        self.assert_expected_environment({"QT_SELECT": "qt5"})

    def assert_expected_environment(self, expected):
        self.assertThat(self.run_mock.call_count, Equals(3))
        for call_args in self.run_mock.call_args_list:
            environment = call_args[1]["env"]
            for variable, value in expected.items():
                self.assertTrue(
                    variable in environment,
                    "Expected variable {!r} to be in environment".format(variable),
                )

                self.assertThat(
                    environment[variable],
                    Equals(value),
                    "Expected ${}={}, but it was {}".format(
                        variable, value, environment[variable]
                    ),
                )
