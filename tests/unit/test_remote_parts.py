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

from unittest import mock

import requests.exceptions
from testtools.matchers import Equals

from snapcraft.internal import errors, remote_parts
from tests import unit


class TestRemoteParts(unit.TestCase):
    @mock.patch("requests.get")
    def test_update_handles_connection_errors(self, mock_requests_get):
        def _fake_get(*args, **kwargs):
            raise requests.exceptions.ConnectionError("I'm a naughty error")

        mock_requests_get.side_effect = _fake_get

        raised = self.assertRaises(
            errors.RemotePartsUpdateConnectionError, remote_parts.update
        )
        self.assertThat(
            raised.message, Equals(requests.exceptions.ConnectionError.__doc__)
        )
