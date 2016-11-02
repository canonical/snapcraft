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

"""This plugin extracts the development files from the stage and packs
   them up to a tar.gz archive

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - exclude:
      (list of strings)
      additional options to pass to the tar command.
    - name:
      string what will be used as name of the tar.gz archive
"""

import os
import snapcraft
import logging
import subprocess

logger = logging.getLogger(__name__)


class DevelopmentPackage(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['exclude'] = {
            'type': 'array',
            'minitems': 0,
            'uniqueItems': True,
            'items': {
                'type': 'string',
            },
            'default': [],
        }
        schema['properties']['extension'] = {
            'type': 'string',
            'default': 'api'
        }
        # The name must be specified
        schema['required'].append('extension')
        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        logger.warning("EXPERIMENTAL: The 'development' plugin's "
                       "functionality is under development and all features"
                       "are experimental.")

    def enable_cross_compilation(self):
        pass

    def build(self):
        super().build()
        self.archive_name = os.getcwd() +\
            "/" +\
            snapcraft.internal.load_config(self.project).data['name'] +\
            "-" +\
            self.options.extension +\
            ".tar.gz"
        for i, p in enumerate(self.options.exclude):
            self.options.exclude[i] = "--exclude={}".format(p.strip())

        if os.path.isdir(self.project.stage_dir):
            self.run(['/bin/tar',
                      'czf',
                      self.archive_name] +
                     self.options.exclude +
                     ['-C',
                      '%s' % self.project.stage_dir, '.'])
        else:
            logger.warning("The stage does not exist")
