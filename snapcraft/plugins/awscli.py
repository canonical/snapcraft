# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd.
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

from snapcraft.plugins.python2 import Python2Plugin

import os
import os.path

class AWSCLIPlugin(Python3Plugin):

    @classmethod
    def schema(cls):
        return {
            '$schema': 'http://json-schema.org/draft-04/schema#',
            'type': 'object',
            'properties': {
                'accesskeyid': {
                    'type': 'string',
                    'default': ''
                },
                'secretaccesskey': {
                    'type': 'string',
                    'default': ''
                },
                'region': {
                    'type': 'string',
                    'default': 'us-east-1'
                },
            },
            'required': ['accesskeyid', 'secretaccesskey']
        }

    def __init__(self, name, options):
        options.pip_packages = ['awscli']
        super().__init__(name, options)
        self.accesskeyid = options.accesskeyid
        self.secretaccesskey = options.secretaccesskey
        self.region = options.region

    def build(self):
        if not super().build():
            return False

        if not self.run(['aws','configure','set','region',self.region]):
            return False
        if not self.run(['aws','configure','set','aws_access_key_id',self.accesskeyid]):
            return False
        if not self.run(['aws','configure','set','aws_secret_access_key',self.secretaccesskey]):
            return False

        #TODO remove hack when two python parts can run at the same time.
        for root, dirs, files in os.walk(self.installdir):
            for name in files:
              if name.endswith('.pyc'):
                  # don't print, instead os.remove
                  os.remove(os.path.join(root,name))
        for pip_bin in ('pip', 'pip3', 'pip3.4'):
            pip_path = os.path.join(self.installdir, 'usr', 'bin', pip_bin)
            if os.path.exists(pip_path):
                os.remove(pip_path)

        return True

    def env(self, root):
        return ['AWS_ACCESS_KEY_ID=%s' % self.accesskeyid,
            'AWS_SECRET_ACCESS_KEY=%s' % self.secretaccesskey]
