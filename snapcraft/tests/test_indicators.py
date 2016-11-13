# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
import progressbar
import requests
# import unittest.mock

from snapcraft import tests
from snapcraft.internal import indicators


class IndicatorsTests(tests.TestCase):

    def test_init_progress_bar_with_length(self):
        pb = indicators._init_progress_bar(10, "destination", "message")
        self.assertEqual(pb.maxval, 10)
        self.assertTrue("message" in pb.widgets)
        pb_widgets_types = [type(w) for w in pb.widgets]
        self.assertTrue(type(progressbar.Bar()) in pb_widgets_types)
        self.assertTrue(type(progressbar.Percentage()) in pb_widgets_types)

    def test_init_progress_bar_with_unknown_length(self):
        pb = indicators._init_progress_bar(0, "destination", "message")
        self.assertEqual(pb.maxval, progressbar.UnknownLength)
        self.assertTrue("message" in pb.widgets)
        pb_widgets_types = [type(w) for w in pb.widgets]
        self.assertTrue(type(progressbar.AnimatedMarker()) in pb_widgets_types)


class IndicatorsDownloadTests(tests.FakeFileHTTPServerBasedTestCase):

    def setUp(self):
        super().setUp()

        dest_dir = 'dst'
        os.makedirs(dest_dir)
        self.file_name = 'snapcraft.yaml'
        self.dest_file = os.path.join(dest_dir, self.file_name)
        self.source = 'http://{}:{}/{file_name}'.format(
            *self.server.server_address, file_name=self.file_name)

    def test_download_request_stream(self):
        request = requests.get(self.source, stream=True, allow_redirects=True)
        indicators.download_requests_stream(request, self.dest_file)

        self.assertTrue(os.path.exists(self.dest_file))

    def test_download_urllib_source(self):
        indicators.download_urllib_source(self.source, self.dest_file)

        self.assertTrue(os.path.exists(self.dest_file))
