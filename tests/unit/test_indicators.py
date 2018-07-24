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

import fixtures
import os
import progressbar
import requests
from unittest.mock import patch

from testtools.matchers import Equals

from snapcraft.internal import indicators
from tests import unit


class DumbTerminalTests(unit.TestCase):
    @patch("os.isatty")
    def setUp(self, mock_os_isatty):
        super().setUp()
        self.mock_os_isatty = mock_os_isatty
        self.mock_os_isatty.return_value = True

    def test_tty_terminal(self):
        self.assertTrue(indicators.is_dumb_terminal())

    def test_not_a_tty_terminal(self):
        self.mock_os_isatty.return_value = False
        self.assertFalse(indicators.is_dumb_terminal())

    def test_dumb_terminal_environment(self):
        self.useFixture(fixtures.EnvironmentVariable("TERM", "dumb"))
        self.assertTrue(indicators.is_dumb_terminal())

    def test_vt100_terminal_environmment(self):
        self.useFixture(fixtures.EnvironmentVariable("TERM", "vt100"))
        self.assertFalse(indicators.is_dumb_terminal())


class ProgressBarInitializationTests(unit.TestCase):

    scenarios = [("Terminal", {"dumb": True}), ("Dumb Terminal", {"dumb": False})]

    @patch("snapcraft.internal.indicators.is_dumb_terminal")
    def test_init_progress_bar_with_length(self, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = self.dumb
        pb = indicators._init_progress_bar(10, "destination", "message")
        self.assertThat(pb.maxval, Equals(10))
        self.assertTrue("message" in pb.widgets)
        pb_widgets_types = [type(w) for w in pb.widgets]
        self.assertTrue(type(progressbar.Percentage()) in pb_widgets_types)
        self.assertThat(
            type(progressbar.Bar()) in pb_widgets_types, Equals(not self.dumb)
        )

    @patch("snapcraft.internal.indicators.is_dumb_terminal")
    def test_init_progress_bar_with_unknown_length(self, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = self.dumb
        pb = indicators._init_progress_bar(0, "destination", "message")
        self.assertThat(pb.maxval, Equals(progressbar.UnknownLength))
        self.assertTrue("message" in pb.widgets)
        pb_widgets_types = [type(w) for w in pb.widgets]
        self.assertThat(
            type(progressbar.AnimatedMarker()) in pb_widgets_types,
            Equals(not self.dumb),
        )


class IndicatorsDownloadTests(unit.FakeFileHTTPServerBasedTestCase):
    def setUp(self):
        super().setUp()

        dest_dir = "dst"
        os.makedirs(dest_dir)
        self.file_name = "snapcraft.yaml"
        self.dest_file = os.path.join(dest_dir, self.file_name)
        self.source = "http://{}:{}/{file_name}".format(
            *self.server.server_address, file_name=self.file_name
        )

    def test_download_request_stream(self):
        request = requests.get(self.source, stream=True, allow_redirects=True)
        indicators.download_requests_stream(request, self.dest_file)

        self.assertTrue(os.path.exists(self.dest_file))

    def test_download_urllib_source(self):
        indicators.download_urllib_source(self.source, self.dest_file)

        self.assertTrue(os.path.exists(self.dest_file))
