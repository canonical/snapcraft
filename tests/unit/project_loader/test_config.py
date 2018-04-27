# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
import subprocess
import sys
import tempfile
import unittest
import unittest.mock
from textwrap import dedent

import fixtures
from testtools.matchers import (Contains, Equals, Is, MatchesRegex, Not,
                                StartsWith)
from testscenarios.scenarios import multiply_scenarios

import snapcraft
from snapcraft.project._project_info import ProjectInfo
from snapcraft.internal import (
    dirs,
    project_loader,
    remote_parts,
)
from snapcraft.internal.project_loader import errors
import snapcraft.internal.project_loader._config as _config

from tests import (
    fixture_setup,
    unit
)


class YamlBaseTestCase(unit.TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        patcher = unittest.mock.patch(
            'snapcraft.internal.project_loader.get_snapcraft_yaml')
        self.mock_get_yaml = patcher.start()
        self.mock_get_yaml.return_value = os.path.join(
            'snap', 'snapcraft.yaml')
        self.addCleanup(patcher.stop)
        validator = project_loader.Validator()
        self.part_schema = validator.part_schema
        self.definitions_schema = validator.definitions_schema
        self.deb_arch = snapcraft.ProjectOptions().deb_arch


class ProjectInfoTestCase(YamlBaseTestCase):

    def setUp(self):
        super().setUp()

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

    def test_properties(self):
        info = ProjectInfo({
            'name': 'foo', 'version': '1',
            'summary': 'bar', 'description': 'baz',
            'confinement': 'strict'
            })
        self.assertThat(info.name, Equals('foo'))
        self.assertThat(info.version, Equals('1'))
        self.assertThat(info.summary, Equals('bar'))
        self.assertThat(info.description, Equals('baz'))
        self.assertThat(info.confinement, Equals('strict'))


class ProjectTestCase(YamlBaseTestCase):

    def setUp(self):
        super().setUp()

        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

    def test_project_with_arguments(self):
        project = snapcraft.project.Project(
            use_geoip=True, parallel_builds=False,
            target_deb_arch='armhf', debug=True)
        self.assertThat(project.use_geoip, Equals(True))
        self.assertThat(project.parallel_builds, Equals(False))
        self.assertThat(project.deb_arch, Equals('armhf'))
        self.assertThat(project.debug, Equals(True))

    def test_project_from_config(self):
        self.make_snapcraft_yaml("""name: foo
version: "1"
summary: bar
description: baz
confinement: strict

parts:
  part1:
    plugin: go
""")

        c = _config.Config()
        project = c._project_options
        self.assertThat(c.data['name'], Equals(project.info.name))
        self.assertThat(c.data['version'], Equals(project.info.version))
        self.assertThat(c.data['summary'], Equals(project.info.summary))
        self.assertThat(c.data['description'],
                        Equals(project.info.description))
        self.assertThat(c.data['confinement'],
                        Equals(project.info.confinement))

        # API of both Project and ProjectOptions must be available
        self.assertTrue(isinstance(project,
                        snapcraft.project.Project))
        self.assertTrue(isinstance(project, snapcraft.ProjectOptions))

    def test_project_from_config_without_summary(self):
        self.make_snapcraft_yaml("""name: foo
version: "1"
description: baz
adopt-info: part1
confinement: strict

parts:
  part1:
    plugin: go
""")

        c = _config.Config()
        project = c._project_options
        self.assertThat(project.info.summary, Equals(None))

    def test_project_from_config_without_description(self):
        self.make_snapcraft_yaml("""name: foo
version: "1"
summary: bar
adopt-info: part1
confinement: strict

parts:
  part1:
    plugin: go
""")

        c = _config.Config()
        project = c._project_options
        self.assertThat(project.info.description, Equals(None))

    def test_project_from_config_without_version(self):
        self.make_snapcraft_yaml("""name: foo
summary: bar
description: baz
adopt-info: part1
confinement: strict

parts:
  part1:
    plugin: go
""")

        c = _config.Config()
        project = c._project_options
        self.assertThat(project.info.version, Equals(None))

    def test_project_passed_to_config(self):
        self.make_snapcraft_yaml("""name: foo
version: "1"
summary: bar
description: baz
confinement: strict

parts:
  part1:
    plugin: go
""")

        project = snapcraft.project.Project()
        c = _config.Config(project)
        self.assertThat(c._project_options, Equals(project))

    def test_no_info_set(self):
        project = snapcraft.project.Project()
        self.assertThat(project.info, Equals(None))

    def test_set_info(self):
        project = snapcraft.project.Project()
        info = ProjectInfo({
            'name': 'foo', 'version': '1',
            'summary': 'bar', 'description': 'baz',
            'confinement': 'strict'
            })
        project.info = info
        self.assertThat(project.info, Equals(info))


class YamlTestCase(YamlBaseTestCase):

    def setUp(self):
        super().setUp()

        patcher = unittest.mock.patch(
            'snapcraft.internal.project_loader._parts_config.PartsConfig'
            '.load_part')
        self.mock_load_part = patcher.start()
        self.addCleanup(patcher.stop)

    def test_yaml_aliases(self,):
        fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict

apps:
  test:
    command: test
    aliases: [test-it, testing]

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        c = _config.Config()
        self.maxDiff = None

        self.assertTrue('aliases' in c.data['apps']['test'],
                        'Expected "aliases" property to be in snap.yaml')
        self.assertThat(
            c.data['apps']['test']['aliases'], Equals(['test-it', 'testing']))

        # Verify that aliases are properly deprecated
        self.assertThat(fake_logger.output, Contains(
            "Aliases are now handled by the store, and shouldn't be declared "
            'in the snap.'))
        self.assertThat(fake_logger.output, Contains(
            "See http://snapcraft.io/docs/deprecation-notices/dn5"))

    def test_yaml_aliases_with_duplicates(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict

apps:
  test1:
    command: test
    aliases: [testing]
  test2:
    command: test
    aliases: [testing]

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        raised = self.assertRaises(errors.DuplicateAliasError,
                                   _config.Config)

        self.assertThat(
            str(raised),
            Equals(
                'Multiple parts have the same alias defined: {!r}'.format(
                    'testing')))

    def test_yaml_invalid_alias(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict

apps:
  test:
    command: test
    aliases: ['.test']  # can't have a non alphanumeric first character

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)
        expected = (
            'The {path!r} property does not match the required schema: '
            '{alias!r} does not match '.format(
                path='apps/test/aliases[0]', alias='.test'
            ))
        self.assertThat(str(raised), Contains(expected))

    def test_config_loads_plugins(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        _config.Config()
        self.mock_load_part.assert_called_with('part1', 'go', {
            'stage-packages': ['fswebcam'],
            'plugin': 'go', 'stage': [], 'prime': [],
        })

    def test_config_composes_with_remote_parts(self):
        self.useFixture(fixture_setup.FakeParts())
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    stage-packages: [fswebcam]
""")

        remote_parts.update()
        _config.Config()

        self.mock_load_part.assert_called_with('part1', 'go', {
            'source': 'http://source.tar.gz', 'stage-packages': ['fswebcam'],
            'plugin': 'go', 'stage': [], 'prime': []})

    def test_config_composes_with_remote_subpart(self):
        self.useFixture(fixture_setup.FakeParts())
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    stage-packages: [fswebcam]
""")

        remote_parts.update()
        _config.Config()

        self.mock_load_part.assert_called_with('part1', 'go', {
            'source': 'http://source.tar.gz', 'stage-packages': ['fswebcam'],
            'plugin': 'go', 'stage': [], 'prime': []})

    def test_chaining_remotes_not_locally_declared(self):
        """Test to verify we can load non locally declared chained remotes."""
        self.useFixture(fixture_setup.FakeParts())
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    after: [curl]
  curl:
    after: [long-described-part]
  part2:
    plugin: nil
""")

        remote_parts.update()
        _config.Config()

    def test_config_composes_with_a_non_existent_remote_part(self):
        self.useFixture(fixture_setup.FakeParts())
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  non-existing-part:
    stage-packages: [fswebcam]
""")

        remote_parts.update()

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftPartMissingError,
            _config.Config)
        self.assertThat(raised.part_name, Equals('non-existing-part'))

    def test_config_after_is_an_undefined_part(self):
        self.useFixture(fixture_setup.FakeParts())
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: nil
    after: [non-existing-part]
""")

        remote_parts.update()

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftPartMissingError,
            _config.Config)
        self.assertThat(raised.part_name, Equals('non-existing-part'))

    def test_config_adds_extra_build_tools_when_cross_compiling(self):
        with unittest.mock.patch('platform.machine') as machine_mock, \
             unittest.mock.patch('platform.architecture') as arch_mock:
            arch_mock.return_value = ('64bit', 'ELF')
            machine_mock.return_value = 'x86_64'
            project_options = snapcraft.ProjectOptions(target_deb_arch='armhf')

        yaml = """name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: nil
"""
        self.make_snapcraft_yaml(yaml)
        config = _config.Config(project_options)

        self.assertThat(config.parts.build_tools,
                        Contains('gcc-arm-linux-gnueabihf')),
        self.assertThat(config.parts.build_tools,
                        Contains('libc6-dev-armhf-cross')),

    def test_config_has_no_extra_build_tools_when_not_cross_compiling(self):
        class ProjectOptionsFake(snapcraft.ProjectOptions):
            @property
            def is_cross_compiling(self):
                return False

        yaml = """name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: nil
"""
        self.make_snapcraft_yaml(yaml)
        config = _config.Config(ProjectOptionsFake())

        self.assertThat(config.parts.build_tools, Equals(set()))

    def test_invalid_yaml_missing_name(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(raised.message,
                        Equals("'name' is a required property"))

    def test_get_metadata(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
architectures:
  - build-on: all
    run-on: amd64
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        config = project_loader.load_config()
        metadata = config.get_metadata()
        self.assertThat(metadata['name'], Equals('test'))
        self.assertThat(metadata['version'], Equals('1'))
        self.assertThat(metadata['arch'], Equals(['amd64']))

    def test_get_metadata_version_adopted(self):
        self.make_snapcraft_yaml(dedent("""\
            name: test
            summary: test
            description: nothing
            architectures:
              - build-on: all
                run-on: amd64
            confinement: strict
            grade: stable
            adopt-info: part1

            parts:
              part1:
                plugin: go
                stage-packages: [fswebcam]
            """))
        config = project_loader.load_config()
        metadata = config.get_metadata()
        self.assertThat(metadata['name'], Equals('test'))
        self.assertThat(metadata['version'], Is(None))
        self.assertThat(metadata['arch'], Equals(['amd64']))

    def test_version_script(self):
        self.make_snapcraft_yaml("""name: test
version: "1.0"
version-script: echo $SNAPCRAFT_PROJECT_VERSION-$SNAPCRAFT_PROJECT_GRADE
summary: test
description: nothing
architectures: ['amd64']
confinement: strict
grade: devel

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        config = project_loader.load_config()
        self.assertThat(
            config.data['version-script'], Equals('echo 1.0-devel'))

    def test_invalid_yaml_invalid_name_as_number(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: 1
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(
            raised.message,
            Equals("The 'name' property does not match the required "
                   "schema: snap names need to be strings."))

    def test_invalid_yaml_invalid_name_as_list(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: []
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(
            raised.message,
            Equals("The 'name' property does not match the required "
                   "schema: snap names need to be strings."))

    def test_invalid_yaml_invalid_name_as_huge_map(self):
        # making my point about not printing the thing for failed type check
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name:
  - a: &a {a: {a: {a: {a: {a: {a: {a: {a: {a: {a: {a: {a: {a: {}}}}}}}}}}}}}}
  - b: &b [*a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a, *a]
  - c: &c [*b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b, *b]
  - z: "I could go on but time and memory are both too short"
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(
            raised.message,
            Equals("The 'name' property does not match the required "
                   "schema: snap names need to be strings."))

    def test_invalid_yaml_invalid_icon_extension(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
icon: icon.foo
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(raised.message,
                        Equals("'icon' must be either a .png or a .svg"))

    def test_invalid_yaml_missing_icon(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
icon: icon.png
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(raised.message,
                        Equals("Specified icon 'icon.png' does not exist"))

    def test_invalid_yaml_invalid_unicode_chars(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: foobar
version: "1"
summary: test\uffff
description: nothing
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(raised.message, Equals(
            "Invalid character '\\uffff' at position 40 "
            "of snap/snapcraft.yaml: special characters are not allowed"))

    def test_invalid_yaml_invalid_name_chars(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: myapp@me_1.0
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(
            raised.message,
            Equals("The 'name' property does not match the required schema:"
                   " 'myapp@me_1.0' is not a valid snap name. Snap names can"
                   " only use ASCII lowercase letters, numbers, and hyphens,"
                   " and must have at least one letter."))

    def test_tab_in_yaml(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
\tconfinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""")

        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(
            raised.message,
            Equals('found character \'\\t\' that cannot start any token '
                   'on line 5 of snap/snapcraft.yaml'))

    def test_yaml_organize_value_none(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict

parts:
  part1:
    plugin: nil
    organize:
      foo:
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)
        self.assertThat(str(raised), Contains(
            "The 'parts/part1/organize/foo' property does not match the "
            "required schema: None is not of type 'string'"))

    def test_yaml_organize_value_empty(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict

parts:
  part1:
    plugin: nil
    organize:
      foo: ''
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)
        self.assertThat(str(raised), Contains(
            "The 'parts/part1/organize/foo' property does not match the "
            "required schema: '' is too short (minimum length is 1)"))

    def test_config_expands_filesets(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
    filesets:
      wget:
        - /usr/lib/wget.so
        - /usr/bin/wget
      build-wget:
        - /usr/lib/wget.a
    stage:
      - $wget
      - $build-wget
    prime:
      - $wget
      - /usr/share/my-icon.png
""")
        _config.Config()

        self.mock_load_part.assert_called_with('part1', 'go', {
            'prime': ['/usr/lib/wget.so', '/usr/bin/wget',
                      '/usr/share/my-icon.png'],
            'plugin': 'go', 'stage-packages': ['fswebcam'],
            'stage': ['/usr/lib/wget.so', '/usr/bin/wget', '/usr/lib/wget.a'],
        })

    def test_get_prereqs(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  main:
    plugin: nil

  dependent:
    plugin: nil
    after: [main]
""")
        config = _config.Config()

        self.assertFalse(config.parts.get_prereqs('main'))
        self.assertThat(
            config.parts.get_prereqs('dependent'),
            Equals({'main'}))

    def test_get_dependents(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  main:
    plugin: nil

  dependent:
    plugin: nil
    after: [main]
""")
        config = _config.Config()

        self.assertFalse(config.parts.get_dependents('dependent'))
        self.assertThat(
            config.parts.get_dependents('main'),
            Equals({'dependent'}))

    def test_replace_snapcraft_variables(self):
        self.make_snapcraft_yaml("""name: project-name
version: "1"
summary: test
description: test
confinement: strict

parts:
  main:
    plugin: make
    source: $SNAPCRAFT_PROJECT_NAME-$SNAPCRAFT_PROJECT_VERSION
    make-options: [DEP=$SNAPCRAFT_STAGE]
""")
        _config.Config()

        self.mock_load_part.assert_called_with('main', 'make', {
            'source': 'project-name-1',
            'plugin': 'make', 'stage': [], 'prime': [],
            'make-options': ['DEP={}'.format(self.stage_dir)],
        })

    def test_environment(self):
        self.make_snapcraft_yaml("""name: project-name
version: "1"
summary: test
description: test
confinement: strict
environment:
    GLOBAL: "1"
    OTHER: valid-value

apps:
  app1:
    command: app1
    environment:
      LOCALE: C
      PLUGIN_PATH: $SNAP_USER_DATA/plugins

parts:
  main:
    plugin: make
    source: $SNAPCRAFT_PROJECT_NAME-$SNAPCRAFT_PROJECT_VERSION
    make-options: [DEP=$SNAPCRAFT_STAGE]
""")
        config = _config.Config().data

        expected_global_env = {'GLOBAL': '1',
                               'OTHER': 'valid-value'}
        self.assertThat(config['environment'], Equals(expected_global_env))

        expected_app_env = {'LOCALE': 'C',
                            'PLUGIN_PATH': '$SNAP_USER_DATA/plugins'}
        self.assertThat(config['apps']['app1']['environment'],
                        Equals(expected_app_env))

    def test_invalid_environment(self):
        self.make_snapcraft_yaml("""name: project-name
version: "1"
summary: test
description: test
confinement: strict
environment:
    INVALID:
        - 1
        - 2

parts:
  main:
    plugin: make
    source: .
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertRegex(
            raised.message,
            "The 'environment/INVALID' property does not match the required "
            "schema: \[1, 2\].*")

    def test_invalid_yaml_version_too_long(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: 'abcdefghijklmnopqrstuvwxyz1234567' # Max is 32 in the store
summary: test
description: test
confinement: strict
grade: stable
parts:
  part1:
    plugin: nil
""")
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(
            raised.message,
            Equals("The 'version' property does not match the required "
                   "schema: 'abcdefghijklmnopqrstuvwxyz1234567' is too long "
                   '(maximum length is 32)'))

    def test_config_uses_remote_part_from_after(self):
        self.useFixture(fixture_setup.FakeParts())
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    after:
      - curl
    plugin: go
    stage-packages: [fswebcam]
""")

        def load_effect(*args, **kwargs):
            mock_part = unittest.mock.Mock()
            mock_part.code.build_packages = []
            mock_part.deps = []
            mock_part.name = args[0]

            return mock_part

        self.mock_load_part.side_effect = load_effect

        project_options = snapcraft.ProjectOptions()

        remote_parts.update()
        _config.Config(project_options)

        call1 = unittest.mock.call(
            'curl',
            'autotools',
            {'plugin': 'autotools', 'stage': [], 'prime': [],
             'source': 'http://curl.org'})
        call2 = unittest.mock.call(
            'part1',
            'go',
            {'plugin': 'go', 'stage': [], 'prime': [],
             'stage-packages': ['fswebcam']})

        self.mock_load_part.assert_has_calls([call1, call2], any_order=True)


class ValidVersionTestCase(YamlBaseTestCase):
    scenarios = [
        (version, dict(version=version)) for
        version in ['buttered-popcorn', '1.2.3', 'v12.4:1:2~', 'HeLlo', 'v+']
    ]

    def test_valid_version(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml(dedent("""
            name: test
            version: {!r}
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              part1:
                plugin: nil
        """).format(self.version))
        _config.Config()


class InvalidVersionTestCase(YamlBaseTestCase):
    scenarios = [
        (version, dict(version=version)) for
        version in [
            '*', '', ':v', '.v', '+v', '~v', '_v', '-v', 'v:', 'v.', 'v_',
            'v-', 'underscores_are_bad',
        ]
    ]

    def test_invalid_version(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml(dedent("""
            name: test
            version: {!r}
            summary: test
            description: test
            confinement: strict
            grade: stable
            parts:
              part1:
                plugin: nil
        """).format(self.version))
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(
            raised.message,
            Equals("The 'version' property does not match the required "
                   "schema: {!r} is not a valid snap version. Snap versions "
                   "consist of upper- and lower-case alphanumeric characters, "
                   "as well as periods, colons, plus signs, tildes, and "
                   "hyphens. They cannot begin with a period, colon, plus "
                   "sign, tilde, or hyphen. They cannot end with a period, "
                   "colon, or hyphen.".format(self.version)))


class YamlEncodingsTestCase(YamlBaseTestCase):

    scenarios = [
        (encoding, dict(encoding=encoding)) for
        encoding in ['utf-8', 'utf-8-sig', 'utf-16']
    ]

    def setUp(self):
        super().setUp()

        patcher = unittest.mock.patch(
            'snapcraft.internal.project_loader._parts_config.PartsConfig'
            '.load_part')
        self.mock_load_part = patcher.start()
        self.addCleanup(patcher.stop)

    def test_config_loads_with_different_encodings(self):
        content = """name: test
version: "1"
summary: test
description: ñoño test
confinement: strict
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
"""

        self.make_snapcraft_yaml(content, encoding=self.encoding)
        _config.Config()

        self.mock_load_part.assert_called_with('part1', 'go', {
            'plugin': 'go', 'stage-packages': ['fswebcam'],
            'stage': [], 'prime': [],
        })


class YamlLoadPluginTestCase(YamlBaseTestCase):

    def test_config_loop(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  p1:
    plugin: tar-content
    source: .
    after: [p2]
  p2:
    plugin: tar-content
    source: .
    after: [p1]
""")
        raised = self.assertRaises(
            errors.SnapcraftLogicError,
            _config.Config)

        self.assertThat(
            raised.message,
            Equals('circular dependency chain found in parts definition'))


class YamlVCSBuildPackagesTestCase(YamlBaseTestCase):

    scenarios = [
        ('git', dict(
            source='git://github.com/ubuntu-core/snapcraft.git',
            expected_package='git')),
        ('bzr', dict(
            source='lp:ubuntu-push',
            expected_package='bzr')),
        ('tar', dict(
            source=(
                'https://github.com/ubuntu-core/snapcraft/archive/'
                '2.0.1.tar.gz'),
            expected_package=None)),
    ]

    def test_config_adds_vcs_packages_to_build_packages(self):
        yaml_t = """name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    source: {0}
    plugin: autotools
"""

        self.make_snapcraft_yaml(yaml_t.format(self.source))
        c = _config.Config()

        if self.expected_package:
            self.assertTrue(
                self.expected_package in c.parts.build_tools,
                '{} not found in {}'.format(
                    self.expected_package, c.parts.build_tools))


class YamlVCSBuildPackagesFromTypeTestCase(YamlBaseTestCase):

    scenarios = [
        ('git', dict(type_='git', package='git')),
        ('hg', dict(type_='hg', package='mercurial')),
        ('mercurial', dict(type_='mercurial', package='mercurial')),
        ('bzr', dict(type_='bzr', package='bzr')),
        ('tar', dict(type_='tar', package=None)),
        ('svn', dict(type_='svn', package='subversion')),
        ('subversion', dict(type_='subversion', package='subversion')),
    ]

    def test_config_adds_vcs_packages_to_build_packages_from_types(self):
        yaml_t = """name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    source: http://something/somewhere
    source-type: {0}
    plugin: autotools
"""

        self.make_snapcraft_yaml(yaml_t.format(self.type_))
        c = _config.Config()

        if self.package:
            self.assertTrue(
                self.package in c.parts.build_tools,
                '{} not found in {}'.format(
                    self.package, c.parts.build_tools))


class ValidAppNamesYamlTestCase(YamlBaseTestCase):

    scenarios = [
        (name, dict(name=name)) for
        name in [
            '1', 'a', 'aa', 'aaa', 'aaaa', 'Aa', 'aA', '1a', 'a1', '1-a',
            'a-1', 'a-a', 'aa-a', 'a-aa', 'a-b-c', '0a-a', 'a-0a',
        ]
    ]

    def test_yaml_valid_app_names(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

apps:
  {!r}:
    command: foo

parts:
  part1:
    plugin: nil
""".format(self.name))
        c = _config.Config()
        self.assertTrue(self.name in c.data['apps'])


class InvalidAppNamesYamlTestCase(YamlBaseTestCase):

    scenarios = [
        (name, dict(name=name)) for
        name in [
            '', '-', '--', 'a--a', 'a-', 'a ', ' a', 'a a', '日本語', '한글',
            'ру́сский язы́к', 'ໄຂ່​ອີ​ສ​ເຕີ້', ':a', 'a:', 'a:a', '_a', 'a_',
            'a_a',
        ]
    ]

    def test_invalid_yaml_invalid_app_names(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

apps:
  {!r}:
    command: foo

parts:
  part1:
    plugin: nil
""".format(self.name))
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertRegex(
            raised.message,
            "The 'apps' property does not match the required schema: .* is "
            'not a valid app name. App names consist of upper- and lower-case '
            'alphanumeric characters and hyphens')


class InvalidHookNamesYamlTestCase(YamlBaseTestCase):

    scenarios = [
        (name, dict(name=name)) for
        name in [
            '', '-', '--', 'a--a', 'a-', 'a ', ' a', 'a a', '日本語', '한글',
            'ру́сский язы́к', 'ໄຂ່​ອີ​ສ​ເຕີ້', ':a', 'a:', 'a:a', '_a', 'a_',
            'a_a', 'Hi',
        ]
    ]

    def test_invalid_yaml_invalid_hook_names(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict
grade: stable

hooks:
  {!r}:
    plugs: [network]

parts:
  part1:
    plugin: nil
""".format(self.name))
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertRegex(
            raised.message,
            "The 'hooks' property does not match the required schema: .* is "
            'not a valid hook name. Hook names consist of lower-case '
            'alphanumeric characters and hyphens')


class ValidConfinmentTypesYamlTestCase(YamlBaseTestCase):

    scenarios = [(confinement, dict(confinement=confinement)) for
                 confinement in ['strict', 'devmode', 'classic']]

    def test_yaml_valid_confinement_types(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: {}
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(self.confinement))
        c = _config.Config()
        self.assertThat(c.data['confinement'], Equals(self.confinement))


class InvalidConfinementTypesYamlTestCase(YamlBaseTestCase):

    scenarios = [(confinement, dict(confinement=confinement)) for
                 confinement in ['foo', 'strict-', '_devmode']]

    def test_invalid_yaml_invalid_confinement_types(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: {}
grade: stable

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(self.confinement))
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(
            raised.message,
            Equals("The 'confinement' property does not match the required "
                   "schema: '{}' is not one of ['classic', 'devmode', "
                   "'strict']".format(self.confinement)))


class ValidGradeTypesYamlTestCase(YamlBaseTestCase):

    scenarios = [(grade, dict(grade=grade)) for
                 grade in ['stable', 'devel']]

    def test_yaml_valid_grade_types(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict
grade: {}

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(self.grade))
        c = _config.Config()
        self.assertThat(c.data['grade'], Equals(self.grade))


class InvalidGradeTypesYamlTestCase(YamlBaseTestCase):

    scenarios = [(grade, dict(grade=grade)) for
                 grade in ['foo', 'unstable-', '_experimental']]

    def test_invalid_yaml_invalid_grade_types(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
confinement: strict
grade: {}

parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(self.grade))
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertThat(
            raised.message,
            Equals("The 'grade' property does not match the required "
                   "schema: '{}' is not one of ['stable', 'devel']".format(
                       self.grade)))


class ValidEpochsYamlTestCase(YamlBaseTestCase):

    scenarios = [
        ('int 0', {
            'yaml': 0,
            'expected': 0,
        }),
        ('int string 0', {
            'yaml': '"0"',
            'expected': '0',
        }),
        ('1*', {
            'yaml': '1*',
            'expected': '1*',
        }),
        ('"1*"', {
            'yaml': '"1*"',
            'expected': '1*',
        }),
        ('int 1', {
            'yaml': 1,
            'expected': 1,
        }),
        ('inst string 1', {
            'yaml': '"1"',
            'expected': '1',
        }),
        ('400 *', {
            'yaml': '400*',
            'expected': '400*',
        }),
        ('"400*"', {
            'yaml': '"400*"',
            'expected': '400*',
        }),
        ('high int', {
            'yaml': 1234,
            'expected': 1234,
        }),
        ('high int string', {
            'yaml': '"1234"',
            'expected': '1234',
        }),
        ('padded with 0', {
            'yaml': '0001',
            'expected': 1,
        }),
    ]

    def test_yaml_valid_epochs(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
epoch: {}
parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(self.yaml))
        c = _config.Config()
        self.assertThat(c.data['epoch'], Equals(self.expected))


class InvalidEpochsYamlTestCase(YamlBaseTestCase):

    scenarios = [(epoch, dict(epoch=epoch)) for
                 epoch in [
                     '0*',
                     '_',
                     '1-',
                     '1+',
                     '-1',
                     '-1*',
                     'a',
                     '1a',
                     '1**',
                     '"01"',
                     '1.2',
                     '"1.2"',
                     '[1]'
                 ]]

    def test_invalid_yaml_invalid_epochs(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: nothing
epoch: {}
parts:
  part1:
    plugin: go
    stage-packages: [fswebcam]
""".format(self.epoch))
        raised = self.assertRaises(
            errors.YamlValidationError,
            _config.Config)

        self.assertRegex(
            raised.message,
            "The 'epoch' property does not match the required "
            "schema:.*is not a 'epoch' \(epochs are positive integers "
            "followed by an optional asterisk\)")


class ValidArchitecturesYamlTestCase(YamlBaseTestCase):

    yaml_scenarios = [
        ('none', {
            'expected_amd64': ['amd64'],
            'expected_i386': ['i386'],
            'expected_armhf': ['armhf'],
            'yaml': None,
        }),
        ('single string list', {
            'expected_amd64': ['amd64'],
            'expected_i386': ['i386'],
            'expected_armhf': ['armhf'],
            'yaml': '[amd64]',
        }),
        ('multiple string list', {
            'expected_amd64': ['amd64', 'i386'],
            'expected_i386': ['amd64', 'i386'],
            'expected_armhf': ['armhf'],
            'yaml': '[amd64, i386]',
        }),
        ('single object list', {
            'expected_amd64': ['amd64'],
            'expected_i386': ['i386'],
            'expected_armhf': ['armhf'],
            'yaml': dedent("""
                - build-on: [amd64]
                  run-on: [amd64]
            """),
        }),
        ('multiple object list', {
            'expected_amd64': ['amd64'],
            'expected_i386': ['i386'],
            'expected_armhf': ['armhf', 'arm64'],
            'yaml': dedent("""
                - build-on: [amd64]
                  run-on: [amd64]
                - build-on: [i386]
                  run-on: [i386]
                - build-on: [armhf]
                  run-on: [armhf, arm64]
            """),
        }),
        ('omit run-on', {
            'expected_amd64': ['amd64'],
            'expected_i386': ['i386'],
            'expected_armhf': ['armhf'],
            'yaml': dedent("""
                - build-on: [amd64]
            """),
        }),
        ('single build-on string, no list', {
            'expected_amd64': ['amd64'],
            'expected_i386': ['i386'],
            'expected_armhf': ['armhf'],
            'yaml': dedent("""
                - build-on: amd64
            """),
        }),
        ('build- and run-on string, no lists', {
            'expected_amd64': ['amd64'],
            'expected_i386': ['amd64'],
            'expected_armhf': ['armhf'],
            'yaml': dedent("""
                - build-on: i386
                  run-on: amd64
            """),
        }),
        ('build on all', {
            'expected_amd64': ['amd64'],
            'expected_i386': ['amd64'],
            'expected_armhf': ['amd64'],
            'yaml': dedent("""
                - build-on: [all]
                  run-on: [amd64]
            """),
        }),
        ('run on all', {
            'expected_amd64': ['all'],
            'expected_i386': ['i386'],
            'expected_armhf': ['armhf'],
            'yaml': dedent("""
                - build-on: [amd64]
                  run-on: [all]
            """),
        }),
    ]

    arch_scenarios = [
        ('amd64', {'target_arch': 'amd64'}),
        ('i386', {'target_arch': 'i386'}),
        ('armhf', {'target_arch': 'armhf'}),
    ]

    scenarios = multiply_scenarios(yaml_scenarios, arch_scenarios)

    def test_architectures(self):
        snippet = ''
        if self.yaml:
            snippet = 'architectures: {}'.format(self.yaml)
        self.make_snapcraft_yaml(dedent("""\
            name: test
            version: "1"
            summary: test
            description: test
            {}
            parts:
              my-part:
                plugin: nil
        """).format(snippet))

        try:
            c = _config.Config(
                snapcraft.project.Project(target_deb_arch=self.target_arch))

            expected = getattr(self, 'expected_{}'.format(self.target_arch))
            self.assertThat(c.data['architectures'], Equals(expected))
        except errors.YamlValidationError as e:
            self.fail('Expected YAML to be valid, got an error: {}'.format(e))


class InitTestCase(unit.TestCase):

    def test_config_raises_on_missing_snapcraft_yaml(self):
        """Test that an error is raised if snap/snapcraft.yaml is missing"""

        raised = self.assertRaises(
            errors.MissingSnapcraftYamlError,
            _config.Config)

        self.assertThat(str(raised), Contains(
            os.path.join('snap', 'snapcraft.yaml')))

    def test_both_new_and_old_yamls_cause_error(self):
        os.mkdir('snap')
        open(os.path.join('snap', 'snapcraft.yaml'), 'w').close()
        open('snapcraft.yaml', 'w').close()

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            _config.Config)

        self.assertThat(
            str(raised), Equals(
                "Found a 'snap/snapcraft.yaml' and a 'snapcraft.yaml', please "
                "remove one."))

    def test_both_new_and_hidden_yamls_cause_error(self):
        os.mkdir('snap')
        open(os.path.join('snap', 'snapcraft.yaml'), 'w').close()
        open('.snapcraft.yaml', 'w').close()

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            _config.Config)

        self.assertThat(
            str(raised), Equals(
                "Found a 'snap/snapcraft.yaml' and a '.snapcraft.yaml', "
                "please remove one."))

    def test_both_visible_and_hidden_yamls_cause_error(self):
        open('snapcraft.yaml', 'w').close()
        open('.snapcraft.yaml', 'w').close()

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            _config.Config)

        self.assertThat(
            str(raised), Equals(
                "Found a 'snapcraft.yaml' and a '.snapcraft.yaml', please "
                'remove one.'))

    def test_snapcraft_yaml_loads(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  main:
    plugin: nil
""")

        _config.Config()

    def test_old_snapcraft_yaml_loads(self):
        fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  main:
    plugin: nil
""")

        os.rename(os.path.join('snap', 'snapcraft.yaml'), 'snapcraft.yaml')
        os.rmdir('snap')
        _config.Config()

    def test_hidden_snapcraft_yaml_loads(self):
        fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(fake_logger)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  main:
    plugin: nil
""")

        os.rename(os.path.join('snap', 'snapcraft.yaml'), '.snapcraft.yaml')
        os.rmdir('snap')
        _config.Config()


class TestYamlEnvironment(unit.TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: nil
""")

        project_options = snapcraft.ProjectOptions()
        patcher = unittest.mock.patch('snapcraft.ProjectOptions')
        mock_project_options = patcher.start()
        mock_project_options.return_value = project_options
        self.arch = project_options.deb_arch
        self.arch_triplet = project_options.arch_triplet
        self.addCleanup(patcher.stop)

    def test_config_snap_environment(self):
        config = _config.Config()

        lib_paths = [os.path.join(self.prime_dir, 'lib'),
                     os.path.join(self.prime_dir, 'usr', 'lib'),
                     os.path.join(self.prime_dir, 'lib',
                                  self.arch_triplet),
                     os.path.join(self.prime_dir, 'usr', 'lib',
                                  self.arch_triplet)]
        for lib_path in lib_paths:
            os.makedirs(lib_path)

        environment = config.snap_env()
        self.assertTrue(
            'PATH="{0}/usr/sbin:{0}/usr/bin:{0}/sbin:{0}/bin:$PATH"'.format(
                self.prime_dir)
            in environment)

        # Ensure that LD_LIBRARY_PATH is present and it contains only the
        # basics.
        paths = []
        for variable in environment:
            if 'LD_LIBRARY_PATH' in variable:
                these_paths = variable.split('=')[1].strip()
                paths.extend(these_paths.replace('"', '').split(':'))

        self.assertTrue(len(paths) > 0,
                        'Expected LD_LIBRARY_PATH to be in environment')

        expected = (os.path.join(self.prime_dir, i) for i in
                    ['lib', os.path.join('usr', 'lib'),
                     os.path.join('lib', self.arch_triplet),
                     os.path.join('usr', 'lib', self.arch_triplet)])
        for item in expected:
            self.assertTrue(
                item in paths,
                'Expected LD_LIBRARY_PATH in {!r} to include {!r}'.format(
                    paths, item))

    def test_config_snap_environment_with_no_library_paths(self):
        config = _config.Config()

        environment = config.snap_env()
        self.assertTrue(
            'PATH="{0}/usr/sbin:{0}/usr/bin:{0}/sbin:{0}/bin:$PATH"'.format(
                self.prime_dir)
            in environment,
            'Current PATH is {!r}'.format(environment))
        for e in environment:
            self.assertFalse('LD_LIBRARY_PATH' in e,
                             'Current environment is {!r}'.format(e))

    @unittest.mock.patch.object(snapcraft.internal.pluginhandler.PluginHandler,
                                'get_primed_dependency_paths')
    def test_config_snap_environment_with_dependencies(self,
                                                       mock_get_dependencies):
        library_paths = {
            os.path.join(self.prime_dir, 'lib1'),
            os.path.join(self.prime_dir, 'lib2'),
        }
        mock_get_dependencies.return_value = library_paths
        config = _config.Config()

        for lib_path in library_paths:
            os.makedirs(lib_path)

        # Ensure that LD_LIBRARY_PATH is present and it contains the
        # extra dependency paths.
        paths = []
        for variable in config.snap_env():
            if 'LD_LIBRARY_PATH' in variable:
                these_paths = variable.split('=')[1].strip()
                paths.extend(these_paths.replace('"', '').split(':'))

        self.assertTrue(len(paths) > 0,
                        'Expected LD_LIBRARY_PATH to be in environment')

        expected = (os.path.join(self.prime_dir, i) for i in ['lib1', 'lib2'])
        for item in expected:
            self.assertTrue(
                item in paths,
                'Expected LD_LIBRARY_PATH ({!r}) to include {!r}'.format(
                    paths, item))

    @unittest.mock.patch.object(snapcraft.internal.pluginhandler.PluginHandler,
                                'get_primed_dependency_paths')
    def test_config_snap_environment_with_dependencies_but_no_paths(
            self, mock_get_dependencies):
        library_paths = {
            os.path.join(self.prime_dir, 'lib1'),
            os.path.join(self.prime_dir, 'lib2'),
        }
        mock_get_dependencies.return_value = library_paths
        config = _config.Config()

        # Ensure that LD_LIBRARY_PATH is present, but is completey empty since
        # no library paths actually exist.
        for variable in config.snap_env():
            self.assertFalse(
                'LD_LIBRARY_PATH' in variable,
                'Expected no LD_LIBRARY_PATH (got {!r})'.format(variable))

    def test_config_runtime_environment_ld(self):
        # Place a few ld.so.conf files in supported locations. We expect the
        # contents of these to make it into the LD_LIBRARY_PATH.
        mesa_dir = os.path.join(
            self.prime_dir, 'usr', 'lib', 'my_arch', 'mesa')
        os.makedirs(mesa_dir)
        with open(os.path.join(mesa_dir, 'ld.so.conf'), 'w') as f:
            f.write('/mesa')

        mesa_egl_dir = os.path.join(
            self.prime_dir, 'usr', 'lib', 'my_arch', 'mesa-egl')
        os.makedirs(mesa_egl_dir)
        with open(os.path.join(mesa_egl_dir, 'ld.so.conf'), 'w') as f:
            f.write('# Standalone comment\n')
            f.write('/mesa-egl')

        config = _config.Config()
        environment = config.snap_env()

        # Ensure that the LD_LIBRARY_PATH includes all the above paths
        paths = []
        for variable in environment:
            if 'LD_LIBRARY_PATH' in variable:
                these_paths = variable.split('=')[1].strip()
                paths.extend(these_paths.replace('"', '').split(':'))

        self.assertTrue(len(paths) > 0,
                        'Expected LD_LIBRARY_PATH to be in environment')

        expected = (os.path.join(self.prime_dir, i) for i in
                    ['mesa', 'mesa-egl'])
        for item in expected:
            self.assertTrue(item in paths,
                            'Expected LD_LIBRARY_PATH to include "{}"'.format(
                                item))

    def test_config_stage_environment_confinement_classic(self):
        patcher = unittest.mock.patch.object(
            snapcraft.internal.os_release.OsRelease, 'version_codename',
            return_value='xenial')
        patcher.start()
        self.addCleanup(patcher.stop)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: classic
grade: stable
base: core

parts:
  part1:
    plugin: nil
""")
        config = _config.Config()
        part = config.parts.get_part('part1')
        environment = config.parts.build_env_for_part(part, root_part=True)
        self.assertIn(
            'LD_LIBRARY_PATH="$LD_LIBRARY_PATH:{base_core_path}/lib:'
            '{base_core_path}/usr/lib:{base_core_path}/lib/{arch_triplet}:'
            '{base_core_path}/usr/lib/{arch_triplet}"'.format(
                base_core_path=self.base_environment.core_path,
                arch_triplet=self.arch_triplet),
            environment)

    def test_stage_environment_confinement_classic_with_incompat_host(self):
        patcher = unittest.mock.patch.object(
            snapcraft.internal.os_release.OsRelease, 'version_codename',
            return_value='incompatible-fake')
        patcher.start()
        self.addCleanup(patcher.stop)

        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: classic
grade: stable
base: core

parts:
  part1:
    plugin: nil
""")
        config = _config.Config()
        part = config.parts.get_part('part1')
        environment = config.parts.build_env_for_part(part, root_part=True)
        for env_item in environment:
            self.assertThat(env_item, Not(StartsWith('LD_LIBRARY_PATH')))

    def test_stage_environment_confinement_classic_with_incompat_base(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: classic
grade: stable
base: fake-core

parts:
  part1:
    plugin: nil
""")
        config = _config.Config()
        part = config.parts.get_part('part1')
        environment = config.parts.build_env_for_part(part, root_part=True)
        for env_item in environment:
            self.assertThat(env_item, Not(StartsWith('LD_LIBRARY_PATH')))

    def test_config_stage_environment(self):
        paths = [os.path.join(self.stage_dir, 'lib'),
                 os.path.join(self.stage_dir, 'lib',
                              self.arch_triplet),
                 os.path.join(self.stage_dir, 'usr', 'lib'),
                 os.path.join(self.stage_dir, 'usr', 'lib',
                              self.arch_triplet),
                 os.path.join(self.stage_dir, 'include'),
                 os.path.join(self.stage_dir, 'usr', 'include'),
                 os.path.join(self.stage_dir, 'include',
                              self.arch_triplet),
                 os.path.join(self.stage_dir, 'usr', 'include',
                              self.arch_triplet)]
        for path in paths:
            os.makedirs(path)

        config = _config.Config()
        environment = config.stage_env()

        self.assertTrue(
            'PATH="{0}/usr/sbin:{0}/usr/bin:{0}/sbin:{0}/bin:$PATH"'.format(
                self.stage_dir) in environment)
        self.assertTrue(
            'LD_LIBRARY_PATH="$LD_LIBRARY_PATH:{stage_dir}/lib:'
            '{stage_dir}/usr/lib:{stage_dir}/lib/{arch_triplet}:'
            '{stage_dir}/usr/lib/{arch_triplet}"'.format(
                stage_dir=self.stage_dir, arch_triplet=self.arch_triplet)
            in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue(
            'CFLAGS="$CFLAGS -I{stage_dir}/include -I{stage_dir}/usr/include '
            '-I{stage_dir}/include/{arch_triplet} '
            '-I{stage_dir}/usr/include/{arch_triplet}"'.format(
                stage_dir=self.stage_dir, arch_triplet=self.arch_triplet)
            in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue(
            'CPPFLAGS="$CPPFLAGS -I{stage_dir}/include '
            '-I{stage_dir}/usr/include '
            '-I{stage_dir}/include/{arch_triplet} '
            '-I{stage_dir}/usr/include/{arch_triplet}"'.format(
                stage_dir=self.stage_dir, arch_triplet=self.arch_triplet)
            in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue(
            'CXXFLAGS="$CXXFLAGS -I{stage_dir}/include '
            '-I{stage_dir}/usr/include '
            '-I{stage_dir}/include/{arch_triplet} '
            '-I{stage_dir}/usr/include/{arch_triplet}"'.format(
                stage_dir=self.stage_dir, arch_triplet=self.arch_triplet)
            in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue(
            'LDFLAGS="$LDFLAGS -L{stage_dir}/lib -L{stage_dir}/usr/lib '
            '-L{stage_dir}/lib/{arch_triplet} '
            '-L{stage_dir}/usr/lib/{arch_triplet}"'.format(
                stage_dir=self.stage_dir, arch_triplet=self.arch_triplet)
            in environment,
            'Current environment is {!r}'.format(environment))
        self.assertTrue('PERL5LIB="{}/usr/share/perl5/"'.format(
            self.stage_dir) in environment)

    def test_parts_build_env_ordering_with_deps(self):
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: nil
  part2:
    plugin: nil
    after: [part1]
""")

        self.useFixture(fixtures.EnvironmentVariable('PATH', '/bin'))

        self.maxDiff = None
        paths = [os.path.join(self.stage_dir, 'lib'),
                 os.path.join(self.stage_dir, 'lib',
                              self.arch_triplet),
                 os.path.join(self.stage_dir, 'usr', 'lib'),
                 os.path.join(self.stage_dir, 'usr', 'lib',
                              self.arch_triplet),
                 os.path.join(self.stage_dir, 'include'),
                 os.path.join(self.stage_dir, 'usr', 'include'),
                 os.path.join(self.stage_dir, 'include',
                              self.arch_triplet),
                 os.path.join(self.stage_dir, 'usr', 'include',
                              self.arch_triplet),
                 os.path.join(self.parts_dir, 'part1', 'install',
                              'include'),
                 os.path.join(self.parts_dir, 'part1', 'install',
                              'lib'),
                 os.path.join(self.parts_dir, 'part2', 'install',
                              'include'),
                 os.path.join(self.parts_dir, 'part2', 'install',
                              'lib')]
        for path in paths:
            os.makedirs(path)

        config = _config.Config()
        part2 = [part for part in
                 config.parts.all_parts if part.name == 'part2'][0]
        env = config.parts.build_env_for_part(part2)
        env_lines = '\n'.join(['export {}\n'.format(e) for e in env])

        shell_env = {
            'CFLAGS': '-I/user-provided',
            'CXXFLAGS': '-I/user-provided',
            'CPPFLAGS': '-I/user-provided',
            'LDFLAGS': '-L/user-provided',
            'LD_LIBRARY_PATH': '/user-provided',
        }

        def get_envvar(envvar):
            with tempfile.NamedTemporaryFile(mode='w+') as f:
                f.write(env_lines)
                f.write('echo ${}'.format(envvar))
                f.flush()
                output = subprocess.check_output(['/bin/sh', f.name],
                                                 env=shell_env)
            return output.decode(sys.getfilesystemencoding()).strip()

        expected_cflags = (
            '-I/user-provided '
            '-I{parts_dir}/part2/install/include -I{stage_dir}/include '
            '-I{stage_dir}/usr/include '
            '-I{stage_dir}/include/{arch_triplet} '
            '-I{stage_dir}/usr/include/{arch_triplet}'.format(
                parts_dir=self.parts_dir,
                stage_dir=self.stage_dir,
                arch_triplet=self.arch_triplet))
        self.assertThat(get_envvar('CFLAGS'), Equals(expected_cflags))
        self.assertThat(get_envvar('CXXFLAGS'), Equals(expected_cflags))
        self.assertThat(get_envvar('CPPFLAGS'), Equals(expected_cflags))

        self.assertThat(
            get_envvar('LDFLAGS'), Equals(
                '-L/user-provided '
                '-L{parts_dir}/part2/install/lib -L{stage_dir}/lib '
                '-L{stage_dir}/usr/lib -L{stage_dir}/lib/{arch_triplet} '
                '-L{stage_dir}/usr/lib/{arch_triplet}'.format(
                    parts_dir=self.parts_dir,
                    stage_dir=self.stage_dir,
                    arch_triplet=self.arch_triplet)))

        self.assertThat(
            get_envvar('LD_LIBRARY_PATH'), Equals(
                '/user-provided:'
                '{parts_dir}/part2/install/lib:'
                '{stage_dir}/lib:'
                '{stage_dir}/usr/lib:'
                '{stage_dir}/lib/{arch_triplet}:'
                '{stage_dir}/usr/lib/{arch_triplet}:'
                '{stage_dir}/lib:'
                '{stage_dir}/usr/lib:'
                '{stage_dir}/lib/{arch_triplet}:'
                '{stage_dir}/usr/lib/{arch_triplet}'.format(
                    parts_dir=self.parts_dir,
                    stage_dir=self.stage_dir,
                    arch_triplet=self.arch_triplet)))

    def test_parts_build_env_contains_parallel_build_count(self):
        self.useFixture(fixture_setup.FakeProjectOptions(
            parallel_build_count='fortytwo'))
        self.make_snapcraft_yaml("""name: test
version: "1"
summary: test
description: test
confinement: strict
grade: stable

parts:
  part1:
    plugin: nil
""")
        config = _config.Config()
        part1 = [part for part in
                 config.parts.all_parts if part.name == 'part1'][0]
        env = config.parts.build_env_for_part(part1)
        self.assertIn('SNAPCRAFT_PARALLEL_BUILD_COUNT=fortytwo', env)


class ValidationBaseTestCase(unit.TestCase):

    def setUp(self):
        super().setUp()
        dirs.setup_dirs()

        patcher = unittest.mock.patch('os.path.exists')
        self.mock_path_exists = patcher.start()
        self.mock_path_exists.return_value = True
        self.addCleanup(patcher.stop)

        self.data = {
            'name': 'my-package-1',
            'version': '1.0-snapcraft1~ppa1',
            'summary': 'my summary less that 79 chars',
            'description': 'description which can be pretty long',
            'adopt-info': 'part1',
            'parts': {
                'part1': {
                    'plugin': 'project',
                    'parse-info': ['test-metadata-file']
                },
            },
        }


class ValidationTestCase(ValidationBaseTestCase):

    def test_summary_too_long(self):
        self.data['summary'] = 'a' * 80
        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(self.data).validate)

        expected_message = (
            "The 'summary' property does not match the required schema: "
            "'{}' is too long (maximum length is 78)").format(
                self.data['summary'])
        self.assertThat(raised.message, Equals(expected_message),
                        message=self.data)

    def test_apps_required_properties(self):
        self.data['apps'] = {'service1': {}}

        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(self.data).validate)

        expected_message = ("The 'apps/service1' property does not match the "
                            "required schema: 'command' is a required "
                            "property")
        self.assertThat(raised.message, Equals(expected_message),
                        message=self.data)

    def test_schema_file_not_found(self):
        mock_the_open = unittest.mock.mock_open()
        mock_the_open.side_effect = FileNotFoundError()

        with unittest.mock.patch(
                'snapcraft.internal.project_loader._schema.open',
                mock_the_open, create=True):
            raised = self.assertRaises(
                errors.YamlValidationError,
                project_loader.Validator,
                self.data)

        expected_message = ('snapcraft validation file is missing from '
                            'installation path')
        self.assertThat(raised.message, Equals(expected_message))

    def test_icon_missing_is_valid_yaml(self):
        self.mock_path_exists.return_value = False

        project_loader.Validator(self.data).validate()

    def test_valid_app_daemons(self):
        self.data['apps'] = {
            'service1': {'command': 'binary1 start', 'daemon': 'simple'},
            'service2': {
                'command': 'binary2',
                'stop-command': 'binary2 --stop',
                'daemon': 'simple'
            },
            'service3': {
                'command': 'binary3',
                'daemon': 'forking',
            },
            'service4': {
                'command': 'binary4',
                'daemon': 'simple',
                'restart-condition': 'always',
            },
            'service5': {
                'command': 'binary5',
                'daemon': 'notify',
            },
            'service6': {
                'command': 'binary6',
                'post-stop-command': 'binary6 --post-stop',
                'daemon': 'simple'
            },
            'service7': {
                'command': 'binary7',
                'reload-command': 'binary7 --reload',
                'daemon': 'simple'
            },
        }

        project_loader.Validator(self.data).validate()

    def test_invalid_restart_condition(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'daemon': 'simple',
                'restart-condition': 'on-watchdog',
            }
        }

        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(self.data).validate)

        self.assertThat(str(raised), Contains(
            "The 'apps/service1/restart-condition' property does not match "
            "the required schema: 'on-watchdog' is not one of ['on-success', "
            "'on-failure', 'on-abnormal', 'on-abort', 'always', 'never']"))

    def test_both_snap_and_prime_specified(self):
        self.data['parts']['part1']['snap'] = ['foo']
        self.data['parts']['part1']['prime'] = ['bar']

        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(self.data).validate)

        self.assertThat(str(raised), MatchesRegex(
            ".*The 'parts/part1' property does not match the required "
            "schema: .* cannot contain both 'snap' and 'prime' keywords.*"))

    def test_missing_required_property_and_missing_adopt_info(self):
        del self.data['summary']
        del self.data['adopt-info']

        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(self.data).validate)

        expected_message = (
            "'adopt-info' is a required property or 'summary' is a required "
            "property")
        self.assertThat(raised.message, Equals(expected_message),
                        message=self.data)


class OldConflictsWithNewScriptletTestCase(ValidationTestCase):

    old_scriptlet_scenarios = [
        ('prepare', {
            'old_keyword': 'prepare',
            'old_value': ['test-prepare'],
        }),
        ('build', {
            'old_keyword': 'build',
            'old_value': ['test-build'],
        }),
        ('install', {
            'old_keyword': 'install',
            'old_value': ['test-install'],
        }),
    ]

    new_scriptlet_scenarios = [
        ('override-pull', {
            'new_keyword': 'override-pull',
            'new_value': ['test-override-pull'],
        }),
        ('override-build', {
            'new_keyword': 'override-build',
            'new_value': ['test-override-build'],
        }),
        ('override-stage', {
            'new_keyword': 'override-stage',
            'new_value': ['test-override-stage'],
        }),
        ('override-prime', {
            'new_keyword': 'override-prime',
            'new_value': ['test-override-prime'],
        }),
    ]

    scenarios = multiply_scenarios(
        old_scriptlet_scenarios, new_scriptlet_scenarios)

    def test_both_old_and_new_keywords_specified(self):
        self.data['parts']['part1'][self.old_keyword] = self.old_value
        self.data['parts']['part1'][self.new_keyword] = self.new_value

        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(self.data).validate)

        self.assertThat(str(raised), MatchesRegex(
            (".*The 'parts/part1' property does not match the required "
             "schema: Parts cannot contain both {0!r} and 'override-\*' "
             "keywords. Use 'override-build' instead of {0!r}.*").format(
                self.old_keyword)))


class DaemonDependencyTestCase(ValidationBaseTestCase):

    scenarios = [
        ('stop-command', dict(
            option='stop-command',
            value='binary1 --stop',
        )),
        ('post-stop-command', dict(
            option='post-stop-command',
            value='binary1 --post-stop'
        )),
    ]

    def test_daemon_dependency(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                self.option: self.value,
            },
        }
        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(self.data).validate)

        self.assertThat(str(raised), Contains(
            "The 'apps/service1' property does not match the required schema: "
            "'daemon' is a dependency of '{}'".format(self.option)))


class RequiredPropertiesTestCase(ValidationBaseTestCase):

    scenarios = [(key, dict(key=key)) for
                 key in ['name', 'parts']]

    def test_required_properties(self):
        data = self.data.copy()
        del data[self.key]

        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(data).validate)

        expected_message = '\'{}\' is a required property'.format(self.key)
        self.assertThat(raised.message, Equals(expected_message),
                        message=data)


class InvalidNamesTestCase(ValidationBaseTestCase):
    e1 = "not a valid snap name. Snap names can only use ASCII lowercase" \
      " letters, numbers, and hyphens, and must have at least one letter."
    e2 = "not a valid snap name. Snap names cannot start with a hyphen."
    e3 = "not a valid snap name. Snap names cannot end with a hyphen."
    e4 = "not a valid snap name. Snap names cannot have two hyphens in a row."
    e5 = "too long (maximum length is 40)"

    scenarios = [
        # snapcraft's existing unit tests
        ("existing test #1", dict(name='package@awesome', err=e1)),
        ("existing test #2", dict(name='something.another', err=e1)),
        ("existing test #3", dict(name='_hideme', err=e1)),
        ("existing test #4", dict(name='-no', err=e2)),
        ("existing test #5", dict(name='a:a', err=e1)),
        ("existing test #6", dict(name='123', err=e1)),

        # this one manages to fail every validation test except type
        ("order check",
            dict(name="-----------------------------------------", err=e5)),

        # from snapd's unit tests (except those covered by above)
        ("name cannot be empty", dict(name="", err=e1)),
        ("name cannot be too long", dict(
            name="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", err=e5)),
        ("dashes alone are not a name", dict(name="-", err=e1)),
        ("dashes alone are not a name, take 2", dict(name="--", err=e1)),
        ("double dashes in a name are not ok", dict(name="a--a", err=e4)),
        ("name should not end with a dash", dict(name="a-", err=e3)),
        ("name cannot have any spaces in it, #1", dict(name="a ", err=e1)),
        ("name cannot have any spaces in it, #2", dict(name=" a", err=e1)),
        ("name cannot have any spaces in it, #3", dict(name="a a", err=e1)),
        ("a number alone is not a name", dict(name="0", err=e1)),
        ("just numbers and dashes", dict(name="1-2-3", err=e1)),
        ("plain ASCII #1", dict(name="реасе", err=e1)),
        ("plain ASCII #2", dict(name="日本語", err=e1)),
        ("plain ASCII #3", dict(name="한글", err=e1)),
        ("plain ASCII #4", dict(name="ру́сский язы́к", err=e1)),

        # from review-tools', except as covered
        ("stress the regexper", dict(
            name="u-9490371368748654323415773467328453675-", err=e3)),
        ("review-tools bad", dict(name="foo?bar", err=e1)),
        ("review-tools bad1", dict(name="foo/bar", err=e1)),
        ("review-tools bad6", dict(name="foo-Bar", err=e1)),
    ]

    def test_invalid_names(self):
        data = self.data.copy()
        data['name'] = self.name

        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(data).validate)

        expected_message = (
            "The 'name' property does not match the required schema: "
            "{!r} is {}").format(self.name, self.err)
        self.assertThat(raised.message, Equals(expected_message),
                        message=data)


class ValidTypesTestCase(ValidationBaseTestCase):

    scenarios = [(type_, dict(type_=type_)) for
                 type_ in ['app', 'gadget', 'kernel', 'os']]

    def test_valid_types(self):
        data = self.data.copy()
        data['type'] = self.type_
        project_loader.Validator(data).validate()


class InvalidTypesTestCase(ValidationBaseTestCase):

    scenarios = [(type_, dict(type_=type_)) for
                 type_ in ['apps', 'framework', 'platform', 'oem']]

    def test_invalid_types(self):
        data = self.data.copy()
        data['type'] = self.type_

        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(data).validate)

        expected_message = (
            "The 'type' property does not match the required "
            "schema: '{}' is not one of "
            "['app', 'base', 'gadget', 'kernel', 'os']").format(self.type_)
        self.assertThat(raised.message, Equals(expected_message),
                        message=data)


class ValidRestartConditionsTestCase(ValidationBaseTestCase):

    scenarios = [(condition, dict(condition=condition)) for
                 condition in ['always', 'on-success', 'on-failure',
                               'on-abnormal', 'on-abort', 'never']]

    def test_valid_restart_conditions(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'daemon': 'simple',
            }
        }
        self.data['apps']['service1']['restart-condition'] = self.condition
        project_loader.Validator(self.data).validate()


class RefreshModeTestCase(ValidationBaseTestCase):

    refresh_modes = ['endure', 'restart']
    scenarios = [(mode, dict(mode=mode)) for mode in refresh_modes]

    def test_valid_modes(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'daemon': 'simple',
                'refresh-mode': self.mode,
            }
        }
        project_loader.Validator(self.data).validate()

    def test_daemon_missing_errors(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'refresh-mode': self.mode,
            }
        }

        self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(self.data).validate)


class StopModeTestCase(ValidationBaseTestCase):

    stop_modes = ['sigterm', 'sigterm-all', 'sighup', 'sighup-all', 'sigusr1',
                  'sigusr1-all', 'sigusr2', 'sigusr2-all']
    scenarios = [(mode, dict(mode=mode)) for mode in stop_modes]

    def test_valid_modes(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'daemon': 'simple',
                'stop-mode': self.mode,
            }
        }
        project_loader.Validator(self.data).validate()

    def test_daemon_missing_errors(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'stop-mode': self.mode,
            }
        }

        self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(self.data).validate)


class InvalidStopRefreshModesTestCase(ValidationBaseTestCase):

    bad_values = [(mode, dict(mode=mode)) for
                  mode in ['sigterm-bad', '123', '-----']]
    keys = [(k, dict(key=k)) for k in ['stop-mode', 'refresh-mode']]

    scenarios = multiply_scenarios(keys, bad_values)

    def setUp(self):
        super().setUp()
        self.valid_modes = {
            'stop-mode': (
                "['sigterm', 'sigterm-all', 'sighup', 'sighup-all', "
                "'sigusr1', 'sigusr1-all', 'sigusr2', 'sigusr2-all']"
            ),
            'refresh-mode': "['endure', 'restart']"
        }

    def test_invalid_modes(self):
        self.data['apps'] = {
            'service1': {
                'command': 'binary1',
                'daemon': 'simple',
                self.key: self.mode
            }
        }
        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(self.data).validate)

        expected_message = (
            "The 'apps/service1/{}' property does not match the "
            "required schema: '{}' is not one of {}"
        ).format(self.key, self.mode, self.valid_modes[self.key])
        self.assertThat(raised.message, Equals(expected_message),
                        message=self.data)


class InvalidAppNamesTestCase(ValidationBaseTestCase):

    scenarios = [(name, dict(name=name)) for
                 name in ['qwe#rty', 'qwe_rty', 'que rty', 'que  rty']]

    def test_invalid_app_names(self):
        data = self.data.copy()
        data['apps'] = {self.name: {'command': '1'}}

        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(data).validate)

        expected_message = (
            "The 'apps' property does not match the required schema: {!r} is "
            "not a valid app name. App names consist of upper- and lower-case "
            "alphanumeric characters and hyphens. They cannot start or end "
            "with a hyphen.").format(self.name)
        self.assertThat(raised.message, Equals(expected_message),
                        message=data)


class InvalidHookNamesTestCase(ValidationBaseTestCase):

    scenarios = [(name, dict(name=name)) for
                 name in ['qwe#rty', 'qwe_rty', 'que rty', 'que  rty', 'Hi']]

    def test_invalid_app_names(self):
        data = self.data.copy()
        data['hooks'] = {self.name: {'plugs': ['network']}}

        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(data).validate)

        expected_message = (
            "The 'hooks' property does not match the required schema: {!r} is "
            "not a valid hook name. Hook names consist of lower-case "
            "alphanumeric characters and hyphens. They cannot start or end "
            "with a hyphen.").format(
                self.name)
        self.assertThat(raised.message, Equals(expected_message),
                        message=data)


class InvalidPartNamesTestCase(ValidationBaseTestCase):

    scenarios = [(name, dict(name=name)) for
                 name in [
                    'plugins', 'qwe#rty', 'qwe_rty', 'que rty', 'que  rty']]

    def test_invalid_part_names(self):
        data = self.data.copy()
        data['parts'] = {self.name: {'plugin': 'nil'}}

        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(data).validate)

        expected_message = (
            "The 'parts' property does not match the required schema: {!r} is "
            "not a valid part name. Part names consist of lower-case "
            "alphanumeric characters, hyphens, plus signs, and forward "
            "slashes. As a special case, 'plugins' is also not a valid part "
            "name.").format(self.name)
        self.assertThat(raised.message, Equals(expected_message),
                        message=data)


class InvalidArchitecturesTestCase(ValidationBaseTestCase):

    scenarios = [
        ('single string', {
            'architectures': 'amd64',
            'message': "'amd64' is not of type 'array'",
        }),
        ('unknown object properties', {
            'architectures': [{'builds-on': ['amd64'], 'runs-on': ['amd64']}],
            'message': "'build-on' is a required property and additional "
                       "properties are not allowed",
        }),
        ('omit build-on', {
            'architectures': [{'run-on': ['amd64']}],
            'message': "'build-on' is a required property",
        }),
        ('build on all and others', {
            'architectures': [{'build-on': ['amd64', 'all']}],
            'message': "'all' can only be used within 'build-on' by itself, "
                       "not with other architectures",
        }),
        ('run on all and others', {
            'architectures': [
                {'build-on': ['amd64'], 'run-on': ['amd64', 'all']}],
            'message': "'all' can only be used within 'run-on' by itself, "
                       "not with other architectures",
        }),
        ('run on all and more objects', {
            'architectures': [
                {'build-on': ['amd64'], 'run-on': ['all']},
                {'build-on': ['i396'], 'run-on': ['i386']},
            ],
            'message': "one of the items has 'all' in 'run-on', but there are "
                       "2 items: upon release they will conflict. 'all' "
                       "should only be used if there is a single item",
        }),
        ('build on all and more objects', {
            'architectures': [
                {'build-on': ['all']},
                {'build-on': ['i396']},
            ],
            'message': "one of the items has 'all' in 'build-on', but there "
                       "are 2 items: snapcraft doesn't know which one to use. "
                       "'all' should only be used if there is a single item",
        }),
        ('multiple builds run on same arch', {
            'architectures': [
                {'build-on': ['amd64'], 'run-on': ['amd64']},
                {'build-on': ['i396'], 'run-on': ['amd64', 'i386']},
            ],
            'message': "multiple items will build snaps that claim to run on "
                       "'amd64'",
        }),
        ('multiple builds run on same arch with implicit run-on', {
            'architectures': [
                {'build-on': ['amd64']},
                {'build-on': ['i396'], 'run-on': ['amd64', 'i386']},
            ],
            'message': "multiple items will build snaps that claim to run on "
                       "'amd64'",
        }),
        ('mixing forms', {
            'architectures': [
                'amd64',
                {'build-on': ['i386'], 'run-on': ['amd64', 'i386']},
            ],
            'message': 'every item must either be a string or an object',
        }),
        ('build on all run on specific', {
            'architectures': [
                {'build-on': ['all'], 'run-on': ['amd64']},
                {'build-on': ['all'], 'run-on': ['i386']},
            ],
            'message': "one of the items has 'all' in 'build-on', but there "
                       "are 2 items: snapcraft doesn't know which one to use. "
                       "'all' should only be used if there is a single item",
        }),
        ('build on overlap', {
            'architectures': [
                {'build-on': ['amd64', 'i386'], 'run-on': ['i386']},
                {'build-on': ['amd64'], 'run-on': ['amd64']},
            ],
            'message': "'amd64' is present in the 'build-on' of multiple "
                       "items, which means snapcraft doesn't know which "
                       "'run-on' to use when building on that architecture",
        }),
    ]

    def test_invalid_architectures(self):
        data = self.data.copy()
        data['architectures'] = self.architectures

        raised = self.assertRaises(
            errors.YamlValidationError,
            project_loader.Validator(data).validate)

        self.assertThat(
            raised.message, MatchesRegex(
                "The 'architectures.*?' property does not match the required "
                "schema: {}".format(self.message)), message=data)


class TestPluginLoadingProperties(unit.TestCase):

    scenarios = [
        ('slots', dict(property='slots')),
        ('plugs', dict(property='plugs')),
    ]

    def test_loading_properties(self):
        self.make_snapcraft_yaml(dedent("""\
            name: my-package-1
            version: 1.0-snapcraft1~ppa1
            summary: my summary less that 79 chars
            description: description which can be pretty long
            parts:
                part1:
                    plugin: nil
                    {property}: [{property}1]
            """).format(property=self.property))

        raised = self.assertRaises(snapcraft.internal.errors.PluginError,
                                   project_loader.load_config)

        self.assertThat(
            raised.message,
            Equals(
                "properties failed to load for "
                "part1: Additional properties are not allowed ('{}' was "
                "unexpected)".format(self.property)))


class TestFilesets(unit.TestCase):

    def setUp(self):
        super().setUp()

        self.properties = {
            'filesets': {
                '1': ['1', '2', '3'],
                '2': [],
            }
        }

    def test_expand_var(self):
        self.properties['stage'] = ['$1']

        fs = _config._expand_filesets_for('stage', self.properties)
        self.assertThat(fs, Equals(['1', '2', '3']))

    def test_no_expansion(self):
        self.properties['stage'] = ['1']

        fs = _config._expand_filesets_for('stage', self.properties)
        self.assertThat(fs, Equals(['1']))

    def test_invalid_expansion(self):
        self.properties['stage'] = ['$3']

        raised = self.assertRaises(
            errors.SnapcraftLogicError,
            _config._expand_filesets_for,
            'stage', self.properties)

        self.assertThat(str(raised), Contains(
            '\'$3\' referred to in the \'stage\' fileset but it is not '
            'in filesets'))


class SnapcraftEnvTestCase(unit.TestCase):

    def test_string_replacements(self):
        replacements = (
            (
                'no replacement',
                'snapcraft_stage/usr/bin',
                'snapcraft_stage/usr/bin',
            ),
            (
                'replaced start',
                '$SNAPCRAFT_STAGE/usr/bin',
                '{}/usr/bin'.format(self.stage_dir),
            ),
            (
                'replaced between',
                '--with-swig $SNAPCRAFT_STAGE/usr/swig',
                '--with-swig {}/usr/swig'.format(self.stage_dir),
            ),
            (
                'project replacement',
                '$SNAPCRAFT_PROJECT_NAME-$SNAPCRAFT_PROJECT_VERSION',
                'project_name-version',
            ),
        )

        for test_name, subject, expected in replacements:
            self.assertThat(project_loader.replace_attr(
                subject, [('$SNAPCRAFT_PROJECT_NAME', 'project_name'),
                          ('$SNAPCRAFT_PROJECT_VERSION', 'version'),
                          ('$SNAPCRAFT_STAGE', self.stage_dir)]),
                            Equals(expected))

    def test_lists_with_string_replacements(self):
        replacements = (
            (
                'no replacement',
                [
                    'snapcraft_stage/usr/bin',
                    '/usr/bin',
                ],
                [
                    'snapcraft_stage/usr/bin',
                    '/usr/bin',
                ],
            ),
            (
                'replaced start',
                [
                    '$SNAPCRAFT_STAGE/usr/bin',
                    '/usr/bin',
                ],
                [
                    '{}/usr/bin'.format(self.stage_dir),
                    '/usr/bin',
                ],
            ),
            (
                'replaced between',
                [
                    '--without-python',
                    '--with-swig $SNAPCRAFT_STAGE/usr/swig',
                ],
                [
                    '--without-python',
                    '--with-swig {}/usr/swig'.format(self.stage_dir),
                ],
            ),
        )

        for test_name, subject, expected in replacements:
            self.assertThat(project_loader.replace_attr(
                subject, [('$SNAPCRAFT_PROJECT_NAME', 'project_name'),
                          ('$SNAPCRAFT_PROJECT_VERSION', 'version'),
                          ('$SNAPCRAFT_STAGE', self.stage_dir)]),
                            Equals(expected))

    def test_tuples_with_string_replacements(self):
        replacements = (
            (
                'no replacement',
                (
                    'snapcraft_stage/usr/bin',
                    '/usr/bin',
                ),
                [
                    'snapcraft_stage/usr/bin',
                    '/usr/bin',
                ],
            ),
            (
                'replaced start',
                (
                    '$SNAPCRAFT_STAGE/usr/bin',
                    '/usr/bin',
                ),
                [
                    '{}/usr/bin'.format(self.stage_dir),
                    '/usr/bin',
                ],
            ),
            (
                'replaced between',
                (
                    '--without-python',
                    '--with-swig $SNAPCRAFT_STAGE/usr/swig',
                ),
                [
                    '--without-python',
                    '--with-swig {}/usr/swig'.format(self.stage_dir),
                ],
            ),
        )

        for test_name, subject, expected in replacements:
            self.assertThat(project_loader.replace_attr(
                subject, [('$SNAPCRAFT_PROJECT_NAME', 'project_name'),
                          ('$SNAPCRAFT_PROJECT_VERSION', 'version'),
                          ('$SNAPCRAFT_STAGE', self.stage_dir)]),
                            Equals(expected))

    def test_dict_with_string_replacements(self):
        replacements = (
            (
                'no replacement',
                {
                    '1': 'snapcraft_stage/usr/bin',
                    '2': '/usr/bin',
                },
                {
                    '1': 'snapcraft_stage/usr/bin',
                    '2': '/usr/bin',
                },
            ),
            (
                'replaced start',
                {
                    '1': '$SNAPCRAFT_STAGE/usr/bin',
                    '2': '/usr/bin',
                },
                {
                    '1': '{}/usr/bin'.format(self.stage_dir),
                    '2': '/usr/bin',
                },
            ),
            (
                'replaced between',
                {
                    '1': '--without-python',
                    '2': '--with-swig $SNAPCRAFT_STAGE/usr/swig',
                },
                {
                    '1': '--without-python',
                    '2': '--with-swig {}/usr/swig'.format(self.stage_dir),
                },
            ),
        )

        for test_name, subject, expected in replacements:
            self.assertThat(project_loader.replace_attr(
                subject, [('$SNAPCRAFT_PROJECT_NAME', 'project_name'),
                          ('$SNAPCRAFT_PROJECT_VERSION', 'version'),
                          ('$SNAPCRAFT_STAGE', self.stage_dir)]),
                            Equals(expected))

    def test_string_replacement_with_complex_data(self):
        subject = {
            'filesets': {
                'files': [
                    'somefile',
                    '$SNAPCRAFT_STAGE/file1',
                    'SNAPCRAFT_STAGE/really',
                ]
            },
            'configflags': [
                '--with-python',
                '--with-swig $SNAPCRAFT_STAGE/swig',
            ],
        }

        expected = {
            'filesets': {
                'files': [
                    'somefile',
                    '{}/file1'.format(self.stage_dir),
                    'SNAPCRAFT_STAGE/really',
                ]
            },
            'configflags': [
                '--with-python',
                '--with-swig {}/swig'.format(self.stage_dir),
            ],
        }

        self.assertThat(project_loader.replace_attr(
            subject, [('$SNAPCRAFT_PROJECT_NAME', 'project_name'),
                      ('$SNAPCRAFT_PROJECT_VERSION', 'version'),
                      ('$SNAPCRAFT_STAGE', self.stage_dir)]),
                        Equals(expected))
