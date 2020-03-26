# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

from tests import unit


# LP: #1733584
class PythonBaseTestCase(unit.TestCase):  # type: ignore
    def _create_python_binary(self, base_dir):
        python_command_path = os.path.join(base_dir, "usr", "bin", "pythontest")
        os.makedirs(os.path.dirname(python_command_path), exist_ok=True)
        open(python_command_path, "w").close()
        return python_command_path
