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

import testscenarios
import yaml
from testtools.matchers import FileExists

from tests import integration


class NodeJSPluginTestCase(testscenarios.WithScenarios, integration.TestCase):

    scenarios = [
        ("npm", dict(package_manager="npm")),
        ("yarn", dict(package_manager="yarn")),
    ]

    def _set_node_package_manager(self, snapcraft_yaml_file):
        if not self.package_manager:
            return

        with open(snapcraft_yaml_file) as f:
            snapcraft_yaml = yaml.load(f)
        snapcraft_yaml["parts"]["nodejs-part"][
            "node-package-manager"
        ] = self.package_manager
        with open(snapcraft_yaml_file, "w") as f:
            yaml.dump(snapcraft_yaml, f)

    def test_rebuilding_possible(self):
        self.copy_project_to_cwd("nodejs-hello")
        self._set_node_package_manager("snapcraft.yaml")

        self.run_snapcraft("build")
        self.run_snapcraft(["clean", "-s", "build"])
        self.run_snapcraft("build")

    def test_build_with_run_commands(self):
        self.copy_project_to_cwd("nodejs-with-run-commands")
        self._set_node_package_manager("snapcraft.yaml")

        self.run_snapcraft("build")
        part_builddir = os.path.join(self.parts_dir, "nodejs-part", "build")
        self.assertThat(os.path.join(part_builddir, "command-one-run"), FileExists())
        self.assertThat(os.path.join(part_builddir, "command-two-run"), FileExists())

        # Ensure the bin entry makes it to bin in the part's install directory
        part_installdir = os.path.join(self.parts_dir, "nodejs-part", "install")
        print_binary_path = os.path.join(part_installdir, "bin", "node-print")
        self.assertThat(print_binary_path, FileExists())
