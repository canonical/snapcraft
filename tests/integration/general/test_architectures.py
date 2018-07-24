# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
import yaml

from testtools.matchers import Equals

from tests import integration


class ArchitecturesTestCase(integration.TestCase):
    def test_simple_architecture(self):
        self.construct_yaml(architectures=[self.deb_arch])

        self.run_snapcraft("prime")

        with open(os.path.join("prime", "meta", "snap.yaml")) as f:
            y = yaml.load(f)

        self.assertThat(y["architectures"], Equals([self.deb_arch]))

    def test_build_and_run_on_architecture(self):
        self.construct_yaml(
            architectures=[{"build-on": [self.deb_arch], "run-on": [self.deb_arch]}]
        )

        self.run_snapcraft("prime")

        with open(os.path.join("prime", "meta", "snap.yaml")) as f:
            y = yaml.load(f)

        self.assertThat(y["architectures"], Equals([self.deb_arch]))

    def test_default_architecture(self):
        self.construct_yaml(architectures=[{"build-on": ["other-arch"]}])

        self.run_snapcraft("prime")

        with open(os.path.join("prime", "meta", "snap.yaml")) as f:
            y = yaml.load(f)

        self.assertThat(y["architectures"], Equals([self.deb_arch]))
