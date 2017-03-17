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

from unittest import mock

from snapcraft import tests


class GrammarTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('snapcraft.repo.Repo')
        self.repo_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.get_mock = self.repo_mock.return_value.get
        self.unpack_mock = self.repo_mock.return_value.unpack
        self.is_valid_mock = self.repo_mock.return_value.is_valid

        def _is_valid(package_name):
            return 'invalid' not in package_name

        self.is_valid_mock.side_effect = _is_valid
