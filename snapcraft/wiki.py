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
    wiki_parts = None

    def _fetch(self):
        if self.wiki_parts is None:
            raw_content = requests.get(PARTS_URI, params=PARTS_URI_PARAMS)
            content = raw_content.text.strip()

            if content.startswith(_WIKI_OPEN):
                content = content[len(_WIKI_OPEN):].strip()
            if content.endswith(_WIKI_CLOSE):
                content = content[:-len(_WIKI_CLOSE)]

            self.wiki_parts = yaml.load(content)

    def get_part(self, name):
        self._fetch()

        if name in self.wiki_parts:
            return self.wiki_parts[name]
