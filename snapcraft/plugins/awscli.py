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

import os
import os.path
import snapcraft.plugins.python3

class AWSCLIPlugin(snapcraft.plugins.python3.Python3Plugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['accesskeyid'] = {
            'type': 'string',
            'default': ''
        }
        schema['properties']['secretaccesskey'] =  {
            'type': 'string',
            'default': ''
        }
        schema['properties']['region'] =  {
            'type': 'string',
            'default': 'us-east-1'
        }

        schema['required'] = ['accesskeyid', 'secretaccesskey']

        return schema

    def __init__(self, name, options):
        options.source = "https://github.com/aws/aws-cli.git"
        options.source_type = 'git'
        #options.pip_packages = ['py2exe', 'awscli']
        super().__init__(name, options)

    def build(self):
        if not super().build():
            return False

        aws = ['python3', os.path.join(self.installdir, 'usr', 'bin', 'aws')]

        if not self.run(aws + ['configure','set','region',self.options.region]):
            return False
        if not self.run(aws + ['configure','set','aws_access_key_id',self.options.accesskeyid]):
            return False
        if not self.run(aws + ['configure','set','aws_secret_access_key',self.options.secretaccesskey]):
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
        env = super().env(root)
        env.extend(['AWS_ACCESS_KEY_ID=%s' % self.options.accesskeyid,
            'AWS_SECRET_ACCESS_KEY=%s' % self.options.secretaccesskey])
        return env
