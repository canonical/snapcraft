# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import snaps_tests


class BaseExampleConsumerTestCase(snaps_tests.SnapsTestCase):

    base_content_dir = "base-example"
    base_consumer_content_dir = "base-example-consumer"

    def test_base_example(self):
        # build/install the alternative base
        snap_path = self.build_snap(self.base_content_dir)
        self.install_snap(snap_path, "base-example", "1.0")
        # build the consumer
        snap_path = self.build_snap(self.base_consumer_content_dir)
        self.install_snap(snap_path, "base-example-consumer", "1.0")
        # FIXME: once snapd supports bases fully run hello against
        #        the alternative base
