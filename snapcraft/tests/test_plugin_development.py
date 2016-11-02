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
import os
import os.path
import fixtures
import subprocess
import fnmatch
import re

import snapcraft
from snapcraft.plugins import development
from snapcraft.plugins.development import DevelopmentPackage
from snapcraft.tests import TestCase
from snapcraft.main import main

from snapcraft.plugins import development


from unittest import mock


class DevelopmentPackagePluginTestCase(TestCase):
    yaml_template = """name: stage-test
version: 1.0
summary: test stage
description: if the build is succesful the state file will be updated
confinement: strict
grade: stable

parts:
{parts}"""
    libapitest_header = """void hello(void);"""
    libapitest_main = """#include <stdio.h>
void hello(void) {
  printf("Hello, library world.\n");
}"""

    yaml_part = """  stage{:d}:
    plugin: nil"""

    def make_snapcraft_yaml(self, n=1):
        parts = '\n'.join([self.yaml_part.format(i) for i in range(n)])
        super().make_snapcraft_yaml(self.yaml_template.format(parts=parts))

        parts = []
        for i in range(n):
            part_dir = os.path.join(self.parts_dir, 'stage{}'.format(i))
            state_dir = os.path.join(part_dir, 'state')
            parts.append({
                'part_dir': part_dir,
                'state_dir': state_dir,
            })

        return parts

    def setUp(self):
        super().setUp()
        self.project_options = snapcraft.ProjectOptions()

        class Options:
            source = '.'
            extension = 'api'
            exclude = ['*.c', '*.o']

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.BasePlugin.build')
        self.base_build_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_schema(self):
        schema = development.DevelopmentPackage.schema()
        properties = schema['properties']
        self.assertEqual(properties['exclude']['type'],
                         'array',
                         'The exclude is not an array')
        self.assertEqual(properties['extension']['type'],
                         'string',
                         'The extension is not string')

    def test_development_packaging(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        self.make_snapcraft_yaml()
        plugin = development.DevelopmentPackage('development',
                                                self.options,
                                                self.project_options)
        plugin.pull()
        os.makedirs('stage')
        with open(os.path.join('stage', 'libapitest.h'), 'w') as f:
            f.write(self.libapitest_header)
            f.close
        with open(os.path.join('stage', 'libapitest.c'), 'w') as f:
            f.write(self.libapitest_main)
            f.close
        open(os.path.join('stage', 'Makefile'), 'w').close()
        open(os.path.join('stage', 'libapitest.so'), 'w').close()
        plugin.build()
        # Test if the archive file is created
        self.assertTrue(os.path.isfile(plugin.archive_name),
                        'The %s archive was not created' % plugin.archive_name)
        tar_process = subprocess.Popen(["/bin/tar",
                                        "tzf",
                                        "%s" % plugin.archive_name],
                                       stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE)
        stdout_value, stderr_value = tar_process.communicate()

        # Test if the package is a valid tar.gz file
        self.assertFalse(tar_process.returncode,
                         'The %s archive is broken' % plugin.archive_name)

        tar_content = stdout_value.decode("utf-8").split('\n')
        for fname in tar_content:
            for pattern in self.options.exclude:
                # Test if there is file in the archive what should be excluded
                self.assertFalse(fnmatch.fnmatch(fname,
                                                 re.sub('--exclude=',
                                                        '',
                                                        pattern)),
                                 'Excluded file in archive')

    def test_stage_defaults(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        parts = self.make_snapcraft_yaml()

        main(['stage'])

        self.assertTrue(os.path.exists(self.stage_dir),
                        'Expected a stage directory')
        self.assertTrue(os.path.exists(self.parts_dir),
                        'Expected a parts directory')
        self.assertTrue(os.path.exists(parts[0]['part_dir']),
                        'Expected a part directory for the build0 part')

        self.verify_state('build0', parts[0]['state_dir'], 'stage')

    def test_dump_enable_cross_compilation(self):
        plugin = DevelopmentPackage('development',
                                    self.options,
                                    self.project_options)
        plugin.enable_cross_compilation()
