# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

import json

import requests


class FakeResponse(requests.Response):
    """A fake requests.Response."""

    def __init__(self, content, status_code):  # pylint: disable=super-init-not-called
        self._content = content
        self.status_code = status_code

    @property
    def content(self):
        return self._content

    @property
    def ok(self):
        return self.status_code == 200

    def json(self, **kwargs):
        return json.loads(self._content)  # type: ignore

    @property
    def reason(self):
        return self._content

    @property
    def text(self):
        return self.content
