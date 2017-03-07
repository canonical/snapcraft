# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

import time

import snaps_tests


class MosquittoTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = 'mosquitto'

    def test_mosquitto(self):
        snap_path = self.build_snap(self.snap_content_dir)
        snap_name = 'mosquitto'
        self.install_snap(snap_path, snap_name, '0.1')
        self.assert_service_running(snap_name, 'mosquitto')
        if not snaps_tests.config.get('skip-install', False):
            # The subscriber will exit after the first message.
            process = self.snappy_testbed.run_command_in_background(
                ['/snap/bin/mosquitto.subscribe', 'test-mosquitto-topic'])
            self.addCleanup(process.wait, 30)
            time.sleep(5)
            self.assert_command_in_snappy_testbed(
                ['/snap/bin/mosquitto.publish', 'test-mosquitto-topic',
                 'test-message'], '')
            self.assert_command_in_snappy_testbed(
                ['/snap/bin/mosquitto.publish', 'test-mosquitto-topic',
                 'exit'], '')
            self.assert_command_in_snappy_testbed(
                ['cat', '/home/ubuntu/snap/mosquitto/*/'
                 'mosquitto.subscriber.log'],
                'MQTT subscriber connected.\n'
                "test-mosquitto-topic b'test-message'\n"
                "test-mosquitto-topic b'exit'\n")
