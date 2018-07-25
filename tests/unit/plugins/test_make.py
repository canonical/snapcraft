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
from snapcraft.plugins import make
from tests import unit


class MakePluginTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class Options:
            makefile = None
            make_parameters = []
            make_install_var = "DESTDIR"
            disable_parallel = False
            artifacts = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

    def test_schema(self):
        schema = make.MakePlugin.schema()

        properties = schema["properties"]
        self.assertTrue(
            "makefile" in properties, 'Expected "makefile" to be included in properties'
        )
        self.assertTrue(
            "make-parameters" in properties,
            'Expected "make-parameters" to be included in properties',
        )

        self.assertTrue(
            "make-install-var" in properties,
            'Expected "make-install-var" to be included in properties',
        )

        makefile = properties["makefile"]
        self.assertTrue(
            "type" in makefile, 'Expected "type" to be included in "makefile"'
        )

        makefile_type = makefile["type"]
        self.assertThat(
            makefile_type,
            Equals("string"),
            'Expected "makefile" "type" to be "string", but it '
            'was "{}"'.format(makefile_type),
        )

        make_parameters = properties["make-parameters"]
        self.assertTrue(
            "type" in make_parameters,
            'Expected "type" to be included in "make-parameters"',
        )

        make_parameters_type = make_parameters["type"]
        self.assertThat(
            make_parameters_type,
            Equals("array"),
            'Expected "make-parameters" "type" to be "array", but it '
            'was "{}"'.format(make_parameters_type),
        )

        make_install_var = properties["make-install-var"]
        self.assertTrue(
            "type" in make_install_var,
            'Expected "type" to be included in "make-install-var"',
        )

        make_install_var_type = make_install_var["type"]
        self.assertThat(
            make_install_var_type,
            Equals("string"),
            'Expected "make-install-var" "type" to be "string", but it '
            'was "{}"'.format(make_install_var_type),
        )

        make_install_var_default = make_install_var["default"]
        self.assertThat(
            make_install_var_default,
            Equals("DESTDIR"),
            'Expected "make-install-var" "default" to be "DESTDIR", but it '
            'was "{}"'.format(make_install_var_default),
        )

    def test_get_build_properties(self):
        expected_build_properties = [
            "makefile",
            "make-parameters",
            "make-install-var",
            "artifacts",
        ]
        resulting_build_properties = make.MakePlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    @mock.patch.object(make.MakePlugin, "run")
    def test_build(self, run_mock):
        plugin = make.MakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)

        plugin.build()

        self.assertThat(run_mock.call_count, Equals(2))
        run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2"], env=None),
                mock.call(
                    ["make", "install", "DESTDIR={}".format(plugin.installdir)],
                    env=None,
                ),
            ]
        )

    @mock.patch.object(make.MakePlugin, "run")
    def test_build_disable_parallel(self, run_mock):
        self.options.disable_parallel = True
        plugin = make.MakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)

        plugin.build()

        self.assertThat(run_mock.call_count, Equals(2))
        run_mock.assert_has_calls(
            [
                mock.call(["make", "-j1"], env=None),
                mock.call(
                    ["make", "install", "DESTDIR={}".format(plugin.installdir)],
                    env=None,
                ),
            ]
        )

    @mock.patch.object(make.MakePlugin, "run")
    def test_build_makefile(self, run_mock):
        self.options.makefile = "makefile.linux"
        plugin = make.MakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)

        plugin.build()

        self.assertThat(run_mock.call_count, Equals(2))
        run_mock.assert_has_calls(
            [
                mock.call(["make", "-f", "makefile.linux", "-j2"], env=None),
                mock.call(
                    [
                        "make",
                        "-f",
                        "makefile.linux",
                        "install",
                        "DESTDIR={}".format(plugin.installdir),
                    ],
                    env=None,
                ),
            ]
        )

    @mock.patch.object(make.MakePlugin, "run")
    def test_build_install_var(self, run_mock):
        self.options.make_install_var = "PREFIX"
        plugin = make.MakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)

        plugin.build()

        self.assertThat(run_mock.call_count, Equals(2))
        run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2"], env=None),
                mock.call(
                    ["make", "install", "PREFIX={}".format(plugin.installdir)], env=None
                ),
            ]
        )

    @mock.patch.object(make.MakePlugin, "run")
    def test_build_empty_install_var(self, run_mock):
        self.options.make_install_var = ""
        plugin = make.MakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)

        plugin.build()

        self.assertThat(run_mock.call_count, Equals(2))
        run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2"], env=None),
                mock.call(["make", "install"], env=None),
            ]
        )

    @mock.patch.object(make.MakePlugin, "run")
    @mock.patch("snapcraft.file_utils.link_or_copy_tree")
    @mock.patch("snapcraft.file_utils.link_or_copy")
    def test_build_artifacts(self, link_or_copy_mock, link_or_copy_tree_mock, run_mock):
        self.options.artifacts = ["dir_artifact", "file_artifact"]
        plugin = make.MakePlugin("test-part", self.options, self.project_options)

        os.makedirs(os.path.join(plugin.builddir, "dir_artifact"))

        plugin.build()

        self.assertThat(run_mock.call_count, Equals(1))
        run_mock.assert_has_calls([mock.call(["make", "-j2"], env=None)])
        self.assertThat(link_or_copy_mock.call_count, Equals(1))
        link_or_copy_mock.assert_has_calls(
            [
                mock.call(
                    os.path.join(plugin.builddir, "file_artifact"),
                    os.path.join(plugin.installdir, "file_artifact"),
                )
            ]
        )
        self.assertThat(link_or_copy_tree_mock.call_count, Equals(1))
        link_or_copy_tree_mock.assert_has_calls(
            [
                mock.call(
                    os.path.join(plugin.builddir, "dir_artifact"),
                    os.path.join(plugin.installdir, "dir_artifact"),
                )
            ]
        )

    @mock.patch.object(make.MakePlugin, "run")
    def test_make_with_env(self, run_mock):
        plugin = make.MakePlugin("test-part", self.options, self.project_options)
        os.makedirs(plugin.sourcedir)

        env = {"foo": "bar"}
        plugin.make(env=env)

        self.assertThat(run_mock.call_count, Equals(2))
        run_mock.assert_has_calls(
            [
                mock.call(["make", "-j2"], env=env),
                mock.call(
                    ["make", "install", "DESTDIR={}".format(plugin.installdir)], env=env
                ),
            ]
        )
