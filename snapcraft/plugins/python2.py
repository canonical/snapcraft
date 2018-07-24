# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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

"""The python2 plugin can be used for python 2 based parts.

This plugin is DEPRECATED in favor of the python plugin.

The python2 plugin can be used for python 2 projects where you would
want to do:

    - import python modules with a requirements.txt
    - build a python project that has a setup.py
    - install sources straight from pip

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - requirements:
      (string)
      Path to a requirements.txt file
    - constraints:
      (string)
      Path to a constraints file
    - process-dependency-links:
      (bool; default: false)
      Enable the processing of dependency links in pip, which allow one
      project to provide places to look for another project
    - python-packages:
      (list)
      A list of dependencies to get from PyPI
"""

import logging

from snapcraft.plugins import python

logger = logging.getLogger(__name__)


class Python2Plugin(python.PythonPlugin):
    def __init__(self, name, options, project):
        options.python_version = "python2"
        super().__init__(name, options, project)
        logger.warning(
            "DEPRECATED: The 'python2' plugin's functionality "
            "has been replaced by the 'python' plugin, and it will "
            "soon be removed."
        )
