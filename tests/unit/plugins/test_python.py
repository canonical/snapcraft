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

import collections
import os
from unittest import mock

from testtools.matchers import Equals, HasLength

import snapcraft
from snapcraft.plugins import python
from tests import fixture_setup, unit


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


class BasePythonPluginTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        class Options:
            source = "."
            source_subdir = ""
            requirements = ""
            constraints = ""
            python_version = "python3"
            python_packages = []
            process_dependency_links = False

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch("snapcraft.plugins._python.Pip")
        self.mock_pip = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch.object(python.PythonPlugin, "_setup_tools_install")
        self.mock_setup_tools = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.internal.os_release.OsRelease")
        self.mock_os_release = patcher.start()
        self.addCleanup(patcher.stop)


class PythonPluginTestCase(BasePythonPluginTestCase):
    def test_schema(self):
        schema = python.PythonPlugin.schema()
        expected_requirements = {"type": "string"}
        expected_constraints = {"type": "string"}
        expected_python_packages = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        self.assertDictEqual(
            expected_requirements, schema["properties"]["requirements"]
        )
        self.assertDictEqual(expected_constraints, schema["properties"]["constraints"])
        self.assertDictEqual(
            expected_python_packages, schema["properties"]["python-packages"]
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

    def test_env(self):
        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        expected_env = []
        env = plugin.env("/testpath")
        self.assertListEqual(expected_env, env)

        env_missing_path = plugin.env("/testpath")
        self.assertTrue("PYTHONPATH=/testpath" not in env_missing_path)

    def test_pull_with_setup_py(self):
        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        setup_directories(plugin, self.options.python_version)

        plugin.pull()

        pip_download = self.mock_pip.return_value.download
        pip_download.assert_called_once_with(
            [],
            constraints=None,
            process_dependency_links=False,
            requirements=None,
            setup_py_dir=plugin.sourcedir,
        )

        self.mock_pip.return_value.wheel.assert_not_called()
        self.mock_pip.return_value.install.assert_not_called()

    def test_pull_with_requirements(self):
        self.options.requirements = "requirements.txt"
        self.options.python_packages = ["test", "packages"]

        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        setup_directories(plugin, self.options.python_version)

        requirements_path = os.path.join(plugin.sourcedir, "requirements.txt")
        open(requirements_path, "w").close()

        plugin.pull()

        pip_download = self.mock_pip.return_value.download
        pip_download.assert_called_once_with(
            ["test", "packages"],
            constraints=None,
            process_dependency_links=False,
            requirements={requirements_path},
            setup_py_dir=plugin.sourcedir,
        )

        self.mock_pip.return_value.wheel.assert_not_called()
        self.mock_pip.return_value.install.assert_not_called()

    def test_pull_with_constraints(self):
        self.options.constraints = "constraints.txt"
        self.options.python_packages = ["test", "packages"]

        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        setup_directories(plugin, self.options.python_version)

        constraints_path = os.path.join(plugin.sourcedir, "constraints.txt")
        open(constraints_path, "w").close()

        plugin.pull()

        pip_download = self.mock_pip.return_value.download
        pip_download.assert_called_once_with(
            ["test", "packages"],
            constraints={constraints_path},
            process_dependency_links=False,
            requirements=None,
            setup_py_dir=plugin.sourcedir,
        )

        self.mock_pip.return_value.wheel.assert_not_called()
        self.mock_pip.return_value.install.assert_not_called()

    @mock.patch.object(python.snapcraft.BasePlugin, "build")
    def test_build(self, mock_base_build):
        self.options.requirements = "requirements.txt"
        self.options.constraints = "constraints.txt"
        self.options.python_packages = ["test", "packages"]

        packages = collections.OrderedDict()
        packages["yaml"] = "1.2"
        packages["extras"] = "1.0"
        self.mock_pip.return_value.list.return_value = packages

        self.useFixture(fixture_setup.CleanEnvironment())
        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        setup_directories(plugin, self.options.python_version)

        for file_name in (self.options.requirements, self.options.constraints):
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
            setup_py_dir=plugin.builddir,
        )

        pip_install = self.mock_pip.return_value.install
        pip_install.assert_called_once_with(
            ["foo", "bar"],
            process_dependency_links=False,
            upgrade=True,
            install_deps=False,
        )

    def test_pip_with_url(self):
        self.options.requirements = "https://test.com/requirements.txt"
        self.options.constraints = "http://test.com/constraints.txt"

        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
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
            constraints={self.options.constraints},
            process_dependency_links=False,
            requirements={self.options.requirements},
            setup_py_dir=plugin.sourcedir,
        )

        pip_install = self.mock_pip.return_value.install
        pip_install.assert_called_once_with(
            [], upgrade=True, process_dependency_links=False, install_deps=False
        )

        pip_wheel = self.mock_pip.return_value.wheel
        pip_wheel.assert_called_once_with(
            [],
            constraints={self.options.constraints},
            process_dependency_links=False,
            requirements={self.options.requirements},
            setup_py_dir=plugin.sourcedir,
        )

    def test_fileset_ignores(self):
        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
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
        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        setup_directories(plugin, self.options.python_version)
        plugin.pull()
        plugin.build()

        pip_download = self.mock_pip.return_value.download
        pip_download.assert_called_once_with(
            [],
            constraints=None,
            process_dependency_links=True,
            requirements=None,
            setup_py_dir=plugin.sourcedir,
        )

        pip_install = self.mock_pip.return_value.install
        pip_install.assert_called_once_with(
            [], upgrade=True, process_dependency_links=True, install_deps=False
        )

        pip_wheel = self.mock_pip.return_value.wheel
        pip_wheel.assert_called_once_with(
            [],
            constraints=None,
            process_dependency_links=True,
            requirements=None,
            setup_py_dir=plugin.sourcedir,
        )

    def test_get_manifest_with_python_packages(self):
        packages = collections.OrderedDict()
        packages["testpackage1"] = "1.0"
        packages["testpackage2"] = "1.2"
        self.mock_pip.return_value.list.return_value = packages

        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
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
        self.options.requirements = "requirements.txt"
        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        setup_directories(plugin, self.options.python_version)
        requirements_path = os.path.join(plugin.sourcedir, "requirements.txt")
        with open(requirements_path, "w") as requirements_file:
            requirements_file.write("testpackage1==1.0\n")
            requirements_file.write("testpackage2==1.2")

        plugin.build()

        self.assertThat(
            plugin.get_manifest()["requirements-contents"],
            Equals("testpackage1==1.0\ntestpackage2==1.2"),
        )

    def test_get_manifest_with_local_constraints(self):
        self.options.constraints = "constraints.txt"

        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        setup_directories(plugin, self.options.python_version)
        constraints_path = os.path.join(plugin.sourcedir, "constraints.txt")
        with open(constraints_path, "w") as constraints_file:
            constraints_file.write("testpackage1==1.0\n")
            constraints_file.write("testpackage2==1.2")

        plugin.build()

        self.assertThat(
            plugin.get_manifest()["constraints-contents"],
            Equals("testpackage1==1.0\ntestpackage2==1.2"),
        )

    def test_plugin_stage_packages_python2_xenial(self):
        self.options.python_version = "python2"
        self.mock_os_release.return_value.version_codename.return_value = "xenial"

        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        self.assertThat(plugin.plugin_stage_packages, Equals(["python"]))

    def test_plugin_stage_packages_python3_xenial(self):
        self.options.python_version = "python3"
        self.mock_os_release.return_value.version_codename.return_value = "xenial"

        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        self.assertThat(plugin.plugin_stage_packages, Equals(["python3"]))

    def test_plugin_stage_packages_python2_bionic(self):
        self.options.python_version = "python2"
        self.mock_os_release.return_value.version_codename.return_value = "bionic"

        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        self.assertThat(plugin.plugin_stage_packages, Equals(["python"]))

    def test_plugin_stage_packages_python3_bionic(self):
        self.options.python_version = "python3"
        self.mock_os_release.return_value.version_codename.return_value = "bionic"

        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        self.assertThat(
            plugin.plugin_stage_packages, Equals(["python3", "python3-distutils"])
        )

    def test_no_python_packages_does_nothing(self):
        # This should be an error but given that we default to
        # 'source: .' and now that pip 10 has been released
        # we run into the need of fixing this situation.
        self.mock_pip.return_value.list.return_value = dict()

        self.useFixture(fixture_setup.CleanEnvironment())
        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        setup_directories(plugin, self.options.python_version, create_setup_py=False)

        pip_wheel = self.mock_pip.return_value.wheel
        pip_wheel.return_value = []

        plugin.build()

        # Pip should not attempt to download again in build (only pull)
        pip_download = self.mock_pip.return_value.download
        pip_download.assert_not_called()

        pip_wheel.assert_called_once_with(
            [],
            constraints=None,
            process_dependency_links=False,
            requirements=None,
            setup_py_dir=None,
        )

        pip_install = self.mock_pip.return_value.install
        pip_install.assert_not_called()


class FileMissingPythonPluginTest(BasePythonPluginTestCase):

    scenarios = (
        ("constraints", dict(property="constraints", file_path="constraints.txt")),
        ("requirements", dict(property="requirements", file_path="requirements.txt")),
    )

    def test_constraints_file_missing(self):
        setattr(self.options, self.property, self.file_path)

        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        setup_directories(plugin, self.options.python_version)

        self.assertRaises(python.SnapcraftPluginPythonFileMissing, plugin.pull)


class PythonPluginWithURLTestCase(
    BasePythonPluginTestCase, unit.FakeFileHTTPServerBasedTestCase
):
    def setUp(self):
        super().setUp()
        self.source = "http://{}:{}/{}".format(
            *self.server.server_address, "testfile.txt"
        )

    def test_get_manifest_with_requirements_url(self):
        self.options.requirements = self.source
        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        setup_directories(plugin, self.options.python_version)

        plugin.build()

        self.assertThat(
            plugin.get_manifest()["requirements-contents"], Equals("Test fake file")
        )

    def test_get_manifest_with_constraints_url(self):
        self.options.constraints = self.source
        plugin = python.PythonPlugin("test-part", self.options, self.project_options)
        setup_directories(plugin, self.options.python_version)

        plugin.build()

        self.assertThat(
            plugin.get_manifest()["constraints-contents"], Equals("Test fake file")
        )
