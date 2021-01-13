# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2020 Canonical Ltd
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

import collections
import os
from unittest import mock

import jsonschema
from testtools.matchers import Equals, HasLength

from snapcraft.internal import errors
from snapcraft.plugins.v1 import python
from tests import fixture_setup, unit

from . import PluginsV1BaseTestCase


def setup_directories(plugin, python_version, create_setup_py=True):
    version = "2.7" if python_version == "python2" else "3.5"
    os.makedirs(plugin.sourcedir)
    os.makedirs(plugin.builddir)
    python_home = os.path.join(plugin.installdir, "usr")
    python_lib_path = os.path.join(python_home, "lib", "python" + version)
    python_include_path = os.path.join(python_home, "include", "python" + version)

    os.makedirs(os.path.join(python_lib_path, "dist-packages"))
    os.makedirs(python_include_path)
    if create_setup_py:
        open(os.path.join(plugin.sourcedir, "setup.py"), "w").close()

    site_path = os.path.join(
        plugin.installdir, "lib", "python" + version, "site-packages"
    )
    os.makedirs(site_path)
    with open(os.path.join(python_lib_path, "site.py"), "w") as f:
        f.write(
            "#!/usr/bin/python3\n"
            "# comment\n"
            "ENABLE_USER_SITE = None\n"
            "USER_SITE = None\n"
            "USER_BASE = None\n"
            "# comment\n"
        )


class PythonPluginBaseTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        class Options:
            source = "."
            source_subdir = ""
            requirements = []
            constraints = ""
            python_version = "python3"
            python_packages = []
            process_dependency_links = False

        self.options = Options()

        patcher = mock.patch("snapcraft.plugins.v1._python.Pip")
        self.mock_pip = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch.object(python.PythonPlugin, "_setup_tools_install")
        self.mock_setup_tools = patcher.start()
        self.addCleanup(patcher.stop)


class PythonPluginPropertiesTest(unit.TestCase):
    def test_schema(self):
        schema = python.PythonPlugin.schema()
        expected_requirements = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        expected_constraints = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        expected_python_packages = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        expected_process_dependency_links = {"type": "boolean", "default": False}
        expected_python_version = {
            "type": "string",
            "default": "python3",
            "enum": ["python2", "python3"],
        }

        self.assertDictEqual(
            expected_requirements, schema["properties"]["requirements"]
        )
        self.assertDictEqual(expected_constraints, schema["properties"]["constraints"])
        self.assertDictEqual(
            expected_python_packages, schema["properties"]["python-packages"]
        )
        self.assertDictEqual(
            expected_process_dependency_links,
            schema["properties"]["process-dependency-links"],
        )
        self.assertDictEqual(
            expected_python_version, schema["properties"]["python-version"]
        )

    def test_get_pull_properties(self):
        expected_pull_properties = [
            "requirements",
            "constraints",
            "python-packages",
            "process-dependency-links",
            "python-version",
        ]
        resulting_pull_properties = python.PythonPlugin.get_pull_properties()

        self.assertThat(
            resulting_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)


class PythonPluginSchemaValidationTest(unit.TestCase):
    def test_sources_validation_neither(self):
        schema = self._get_schema()
        properties = {}
        self.assertRaises(
            jsonschema.ValidationError, jsonschema.validate, properties, schema
        )

    def test_sources_validation_source(self):
        schema = self._get_schema()
        properties = {"source": ""}
        jsonschema.validate(properties, schema)

    def test_sources_validation_packages(self):
        schema = self._get_schema()
        properties = {"python-packages": []}
        jsonschema.validate(properties, schema)

    def test_sources_validation_both(self):
        schema = self._get_schema()
        properties = {"source": "", "python-packages": []}
        jsonschema.validate(properties, schema)

    def _get_schema(self):
        schema = python.PythonPlugin.schema()
        # source definition comes from the main schema
        schema["properties"]["source"] = {"type": "string"}
        return schema


class PythonPluginTest(PythonPluginBaseTest):
    def test_env(self):
        plugin = python.PythonPlugin("test-part", self.options, self.project)
        expected_env = []
        env = plugin.env("/testpath")
        self.assertListEqual(expected_env, env)

        env_missing_path = plugin.env("/testpath")
        self.assertTrue("PYTHONPATH=/testpath" not in env_missing_path)

    def test_pull_with_setup_py(self):
        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)

        plugin.pull()

        pip_download = self.mock_pip.return_value.download
        pip_download.assert_called_once_with(
            [],
            constraints=set(),
            process_dependency_links=False,
            requirements=set(),
            setup_py_dir=None,
        )

        self.mock_pip.return_value.wheel.assert_not_called()
        self.mock_pip.return_value.install.assert_not_called()

    def test_pull_with_requirements(self):
        self.options.requirements = ["requirements.txt"]
        self.options.python_packages = ["test", "packages"]

        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)

        requirements_path = os.path.join(plugin.sourcedir, "requirements.txt")
        open(requirements_path, "w").close()

        plugin.pull()

        pip_download = self.mock_pip.return_value.download
        pip_download.assert_called_once_with(
            ["test", "packages"],
            constraints=set(),
            process_dependency_links=False,
            requirements={requirements_path},
            setup_py_dir=None,
        )

        self.mock_pip.return_value.wheel.assert_not_called()
        self.mock_pip.return_value.install.assert_not_called()

    def test_pull_with_constraints(self):
        self.options.constraints = ["constraints.txt"]
        self.options.python_packages = ["test", "packages"]

        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)

        constraints_path = os.path.join(plugin.sourcedir, "constraints.txt")
        open(constraints_path, "w").close()

        plugin.pull()

        pip_download = self.mock_pip.return_value.download
        pip_download.assert_called_once_with(
            ["test", "packages"],
            constraints={constraints_path},
            process_dependency_links=False,
            requirements=set(),
            setup_py_dir=None,
        )

        self.mock_pip.return_value.wheel.assert_not_called()
        self.mock_pip.return_value.install.assert_not_called()

    @mock.patch.object(python.PluginV1, "build")
    def test_build(self, mock_base_build):
        self.options.requirements = ["requirements.txt"]
        self.options.constraints = ["constraints.txt"]
        self.options.python_packages = ["test", "packages"]

        packages = collections.OrderedDict()
        packages["yaml"] = "1.2"
        packages["extras"] = "1.0"
        self.mock_pip.return_value.list.return_value = packages

        self.useFixture(fixture_setup.CleanEnvironment())
        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)

        for file_name in self.options.requirements + self.options.constraints:
            path = os.path.join(plugin.sourcedir, file_name)
            open(path, "w").close()

        requirements_path = os.path.join(plugin.builddir, "requirements.txt")
        constraints_path = os.path.join(plugin.builddir, "constraints.txt")

        def build_side_effect():
            open(os.path.join(plugin.builddir, "setup.py"), "w").close()
            os.mkdir(os.path.join(plugin.builddir, "dist"))
            open(os.path.join(plugin.builddir, "dist", "package.tar"), "w").close()
            open(requirements_path, "w").close()
            open(constraints_path, "w").close()

        mock_base_build.side_effect = build_side_effect

        pip_wheel = self.mock_pip.return_value.wheel
        pip_wheel.return_value = ["foo", "bar"]

        plugin.build()

        # Pip should not attempt to download again in build (only pull)
        pip_download = self.mock_pip.return_value.download
        pip_download.assert_not_called()

        pip_wheel.assert_called_once_with(
            ["test", "packages"],
            constraints={constraints_path},
            process_dependency_links=False,
            requirements={requirements_path},
            setup_py_dir=None,
        )

        pip_install = self.mock_pip.return_value.install
        self.assertThat(pip_install.call_count, Equals(2))
        pip_install.assert_has_calls(
            [
                mock.call(
                    ["foo", "bar"],
                    process_dependency_links=False,
                    upgrade=True,
                    install_deps=False,
                ),
                mock.call(
                    [],
                    setup_py_dir=plugin.builddir,
                    constraints={constraints_path},
                    process_dependency_links=False,
                    upgrade=True,
                ),
            ]
        )

    def test_pip_with_url(self):
        self.options.requirements = ["https://test.com/requirements.txt"]
        self.options.constraints = ["http://test.com/constraints.txt"]

        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)

        # Patch requests so we don't hit the network when the requirements
        # and constraints files are downloaded to save their contents in the
        # manifest.
        with mock.patch("requests.get"):
            plugin.pull()
            plugin.build()

        pip_download = self.mock_pip.return_value.download
        pip_download.assert_called_once_with(
            [],
            constraints=set(self.options.constraints),
            process_dependency_links=False,
            requirements=set(self.options.requirements),
            setup_py_dir=None,
        )

        pip_install = self.mock_pip.return_value.install
        self.assertThat(pip_install.call_count, Equals(2))
        pip_install.assert_has_calls(
            [
                mock.call(
                    [], upgrade=True, process_dependency_links=False, install_deps=False
                ),
                mock.call(
                    [],
                    setup_py_dir=plugin.sourcedir,
                    constraints=set(self.options.constraints),
                    process_dependency_links=False,
                    upgrade=True,
                ),
            ]
        )

        pip_wheel = self.mock_pip.return_value.wheel
        pip_wheel.assert_called_once_with(
            [],
            constraints=set(self.options.constraints),
            process_dependency_links=False,
            requirements=set(self.options.requirements),
            setup_py_dir=None,
        )

    def test_fileset_ignores(self):
        plugin = python.PythonPlugin("test-part", self.options, self.project)
        expected_fileset = [
            "-bin/pip",
            "-bin/pip2",
            "-bin/pip3",
            "-bin/pip2.7",
            "-bin/pip3.*",
            "-bin/easy_install*",
            "-bin/wheel",
            "-**/__pycache__",
            "-**/*.pyc",
            "-lib/python*/site-packages/*/RECORD",
        ]
        fileset = plugin.snap_fileset()
        self.assertListEqual(expected_fileset, fileset)

    def test_process_dependency_links(self):
        self.options.process_dependency_links = True
        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)
        plugin.pull()
        plugin.build()

        pip_download = self.mock_pip.return_value.download
        pip_download.assert_called_once_with(
            [],
            constraints=set(),
            process_dependency_links=True,
            requirements=set(),
            setup_py_dir=None,
        )

        pip_install = self.mock_pip.return_value.install
        self.assertThat(pip_install.call_count, Equals(2))
        pip_install.assert_has_calls(
            [
                mock.call(
                    [], upgrade=True, process_dependency_links=True, install_deps=False
                ),
                mock.call(
                    [],
                    setup_py_dir=plugin.sourcedir,
                    constraints=set(),
                    process_dependency_links=True,
                    upgrade=True,
                ),
            ]
        )

        pip_wheel = self.mock_pip.return_value.wheel
        pip_wheel.assert_called_once_with(
            [],
            constraints=set(),
            process_dependency_links=True,
            requirements=set(),
            setup_py_dir=None,
        )

    def test_get_manifest_with_python_packages(self):
        packages = collections.OrderedDict()
        packages["testpackage1"] = "1.0"
        packages["testpackage2"] = "1.2"
        self.mock_pip.return_value.list.return_value = packages

        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)
        plugin.build()
        self.assertThat(
            plugin.get_manifest(),
            Equals(
                collections.OrderedDict(
                    {"python-packages": ["testpackage1=1.0", "testpackage2=1.2"]}
                )
            ),
        )

    def test_get_manifest_with_local_requirements(self):
        self.options.requirements = ["requirements.txt"]
        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)
        requirements_path = os.path.join(plugin.sourcedir, "requirements.txt")
        with open(requirements_path, "w") as requirements_file:
            requirements_file.write("testpackage1==1.0\n")
            requirements_file.write("testpackage2==1.2")

        plugin.build()

        self.assertThat(
            plugin.get_manifest()["requirements-contents"],
            Equals(["testpackage1==1.0", "testpackage2==1.2"]),
        )

    def test_get_manifest_with_multiple_local_requirements(self):
        self.options.requirements = ["requirements1.txt", "requirements2.txt"]
        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)
        requirements1_path = os.path.join(plugin.sourcedir, "requirements1.txt")
        requirements2_path = os.path.join(plugin.sourcedir, "requirements2.txt")
        with open(requirements1_path, "w") as requirements_file:
            requirements_file.write("testpackage1==1.0\n")
        with open(requirements2_path, "w") as requirements_file:
            requirements_file.write("testpackage2==1.2")

        plugin.build()

        self.assertThat(
            plugin.get_manifest()["requirements-contents"],
            Equals(["testpackage1==1.0", "testpackage2==1.2"]),
        )

    def test_get_manifest_with_local_constraints(self):
        self.options.constraints = ["constraints.txt"]

        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)
        constraints_path = os.path.join(plugin.sourcedir, "constraints.txt")
        with open(constraints_path, "w") as constraints_file:
            constraints_file.write("testpackage1==1.0\n")
            constraints_file.write("testpackage2==1.2")

        plugin.build()

        self.assertThat(
            plugin.get_manifest()["constraints-contents"],
            Equals(["testpackage1==1.0", "testpackage2==1.2"]),
        )

    def test_get_manifest_with_multiple_local_constraints(self):
        self.options.constraints = ["constraints1.txt", "constraints2.txt"]

        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)
        constraints1_path = os.path.join(plugin.sourcedir, "constraints1.txt")
        with open(constraints1_path, "w") as constraints_file:
            constraints_file.write("testpackage1==1.0\n")
        constraints2_path = os.path.join(plugin.sourcedir, "constraints2.txt")
        with open(constraints2_path, "w") as constraints_file:
            constraints_file.write("testpackage2==1.2")

        plugin.build()

        self.assertThat(
            plugin.get_manifest()["constraints-contents"],
            Equals(["testpackage1==1.0", "testpackage2==1.2"]),
        )

    def test_no_python_packages_does_nothing(self):
        # This should be an error but given that we default to
        # 'source: .' and now that pip 10 has been released
        # we run into the need of fixing this situation.
        self.mock_pip.return_value.list.return_value = dict()

        self.useFixture(fixture_setup.CleanEnvironment())
        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version, create_setup_py=False)

        pip_wheel = self.mock_pip.return_value.wheel
        pip_wheel.return_value = []

        plugin.build()

        # Pip should not attempt to download again in build (only pull)
        pip_download = self.mock_pip.return_value.download
        pip_download.assert_not_called()

        pip_wheel.assert_called_once_with(
            [],
            constraints=set(),
            process_dependency_links=False,
            requirements=set(),
            setup_py_dir=None,
        )

        pip_install = self.mock_pip.return_value.install
        pip_install.assert_not_called()


class PythonCore18Test(PythonPluginBaseTest):
    def test_plugin_stage_packages_python2(self):
        self.options.python_version = "python2"

        plugin = python.PythonPlugin("test-part", self.options, self.project)
        self.assertThat(plugin.plugin_stage_packages, Equals(["python"]))

    def test_plugin_stage_packages_python3(self):
        self.options.python_version = "python3"

        plugin = python.PythonPlugin("test-part", self.options, self.project)
        self.assertThat(
            plugin.plugin_stage_packages, Equals(["python3", "python3-distutils"])
        )


class PythonCoreTest(PythonPluginBaseTest):
    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = "core"

    def test_plugin_stage_packages_python2(self):
        self.options.python_version = "python2"

        plugin = python.PythonPlugin("test-part", self.options, self.project)
        self.assertThat(plugin.plugin_stage_packages, Equals(["python"]))

    def test_plugin_stage_packages_python3(self):
        self.options.python_version = "python3"

        plugin = python.PythonPlugin("test-part", self.options, self.project)
        self.assertThat(plugin.plugin_stage_packages, Equals(["python3"]))


class FileMissingPythonPluginTest(PythonPluginBaseTest):
    def test_constraints_file_missing(self):
        self.options.constraints = "constraints.txt"

        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)

        self.assertRaises(python.SnapcraftPluginPythonFileMissing, plugin.pull)

    def test_requirements_file_missing(self):
        self.options.requirements = "requirements.txt"

        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)

        self.assertRaises(python.SnapcraftPluginPythonFileMissing, plugin.pull)


class PythonPluginWithURLTestCase(
    PythonPluginBaseTest, unit.FakeFileHTTPServerBasedTestCase
):
    def setUp(self):
        super().setUp()
        self.source = "http://{}:{}/{}".format(
            *self.server.server_address, "testfile.txt"
        )

    def test_get_manifest_with_requirements_url(self):
        self.options.requirements = [self.source]
        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)

        plugin.build()

        self.assertThat(
            plugin.get_manifest()["requirements-contents"], Equals(["Test fake file"])
        )

    def test_get_manifest_with_constraints_url(self):
        self.options.constraints = [self.source]
        plugin = python.PythonPlugin("test-part", self.options, self.project)
        setup_directories(plugin, self.options.python_version)

        plugin.build()

        self.assertThat(
            plugin.get_manifest()["constraints-contents"], Equals(["Test fake file"])
        )


class PythonPluginUnsupportedBase(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = "unsupported-base"

        class Options:
            source = "dir"

        self.options = Options()

    def test_unsupported_base_raises(self):
        self.assertRaises(
            errors.PluginBaseError,
            python.PythonPlugin,
            "test-part",
            self.options,
            self.project,
        )
