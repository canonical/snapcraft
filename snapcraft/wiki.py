# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

import logging
import requests
import yaml

PARTS_URI = 'https://wiki.ubuntu.com/Snappy/Parts'
PARTS_URI_PARAMS = {'action': 'raw'}

_WIKI_OPEN = '{{{'
_WIKI_CLOSE = '}}}'

logging.getLogger("urllib3").setLevel(logging.CRITICAL)


class Wiki:

    def __init__(self):
        self.wiki_parts = self._fetch()

    def _fetch(self):
        raw_content = requests.get(PARTS_URI, params=PARTS_URI_PARAMS)
        content = raw_content.text.strip()

        if content.startswith(_WIKI_OPEN):
            content = content[len(_WIKI_OPEN):].strip()
        if content.endswith(_WIKI_CLOSE):
            content = content[:-len(_WIKI_CLOSE)]

        return yaml.load(content)

    def get_part(self, name):
        try:
            return self.wiki_parts[name]
        except KeyError:
            return None
