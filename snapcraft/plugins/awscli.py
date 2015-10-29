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

"""This plugin brings in the AWS command line interface and configures it.

The AWS command line can be used for a variety of things including accessing
s3 or setting up ec2 services. It also is used for setting IoT things. This
plugin takes configuration values for your AWS keys, downloads and installs
the client into your snap, and configures it with those keys.

Settings for this plugin include:

    - accesskeyid:
      (string)
      AWS Access Key to use
    - secreetaccesskey:
      (string)
      AWS Secret Key
    - region
      (string)
      Region of EC2 to use, defaults to us-east-1

It is important to not that the key will be stored in the snap itself and
could be harvested with appropriate tools.
"""

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
        schema['properties']['secretaccesskey'] = {
            'type': 'string',
            'default': ''
        }
        schema['properties']['region'] = {
            'type': 'string',
            'default': 'us-east-1'
        }

        schema['required'] = ['accesskeyid', 'secretaccesskey']

        return schema

    def __init__(self, name, options):
        options.source = "https://github.com/aws/aws-cli.git"
        options.source_type = 'git'
        super().__init__(name, options)

    def build(self):
        super().build()

        aws = ['python3', os.path.join(self.installdir, 'usr', 'bin', 'aws')]

        self.run(aws + ['configure',
                        'set', 'region', self.options.region])
        self.run(aws + ['configure',
                        'set', 'aws_access_key_id', self.options.accesskeyid])
        self.run(aws + ['configure',
                        'set', 'aws_secret_access_key',
                        self.options.secretaccesskey])

    def env(self, root):
        env = super().env(root)
        env.extend(['AWS_ACCESS_KEY_ID=%s' % self.options.accesskeyid,
                    'AWS_SECRET_ACCESS_KEY=%s' % self.options.secretaccesskey])
        return env

    def snap_fileset(self):
        fileset = super().snap_fileset()
        fileset.append('-usr/lib/python3/dist-packages/easy-install.pth')
        fileset.append('-usr/lib/python3.4/__pycache__/')
        fileset.append('-usr/lib/python3.4/encodings/__pycache__/')
        fileset.append('-usr/lib/python3.4/plat*/__pycache__/')
        return fileset
