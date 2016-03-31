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

"""The script plugin is useful for parts whose build is defined in a script

The use case here is if you have a shell script which defines some generic
steps that fit to no other tool and produces a result after them.

All the scripts called by this plugin have to accept at least one argument.
This first argument defines the directory into which the resulting files should
be copied by the script.

Additionally, this plugin uses the following plugin-specific keywords:

    - script:
      (string)
      path to the script file to execute
    - destination-dir:
      (string)
      path relative to the install directory where results should be placed
"""

import os
import shutil
import snapcraft


class ScriptPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        return {
            '$schema': 'http://json-schema.org/draft-04/schema#',
            'type': 'object',
            'properties': {
                'script': {
                    'type': 'string',
                },
                'destination-dir': {
                    'type': 'string',
                    'default': '',
                },
            },
            'required': [
                'script',
            ]
        }

    def build(self):
        # Assert build dir exists and copy script there
        script_file_name = os.path.basename(self.options.script)
        if os.path.exists(self.builddir):
            shutil.rmtree(self.builddir)
        os.makedirs(self.builddir)
        shutil.copy(
            os.path.join(os.getcwd(), self.options.script),
            self.builddir
        )
        # Determine destination directory and make sure it exists
        dst = self.installdir
        if self.options.destination_dir != '':
            dst = os.path.join(self.installdir, self.options.destination_dir)
        if os.path.exists(dst):
            shutil.rmtree(dst)
        os.makedirs(dst)
        # Run the script inside the build directory.
        # Note the script itself should copy its results to dst
        return self.run(
            [
                'sh',
                os.path.join(self.builddir, script_file_name),
                dst
            ],
            self.builddir
        )
