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

from snapcraft.plugins.python3 import Python3Plugin

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
        super().__init__(name, options)
        self.accesskeyid = options.accesskeyid
        self.secretaccesskey = options.secretaccesskey
        self.region = options.region

    def build(self):
        easy_install = os.path.join(
            self.installdir, 'usr', 'bin', 'easy_install3')
        prefix = os.path.join(self.installdir, 'usr')
        site_packages_dir = os.path.join(
            prefix, 'lib', self.python_version, 'site-packages')

        if not os.path.exists(site_packages_dir):
            os.symlink(
                os.path.join(prefix, 'lib', 'python3', 'dist-packages'),
                site_packages_dir)

        if not self.run(['python3', easy_install, '--prefix', prefix, 'pip']):
            return False

        pip3 = os.path.join(self.installdir, 'usr', 'bin', 'pip3')
        pip_install = ['python3', pip3, 'install','--no-compile', '--target',
                       site_packages_dir]

        if not self.run(pip_install + ['--upgrade','awscli', ]):
            return False

        self.run(['aws','configure','set','region',self.region])
        self.run(['aws','configure','set','aws_access_key_id',self.accesskeyid])
        self.run(['aws','configure','set','aws_secret_access_key',self.secretaccesskey])

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

        #return super().build()
        return True

    def env(self, root):
        return ['AWS_ACCESS_KEY_ID=%s' % self.accesskeyid,
            'AWS_SECRET_ACCESS_KEY=%s' % self.secretaccesskey]
