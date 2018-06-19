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

from textwrap import dedent

from testtools.matchers import Contains, Equals, Is

from . import LoadPartBaseTest, ProjectLoaderBaseTest
from snapcraft.internal import project_loader

from snapcraft.internal.project_loader import errors
import snapcraft.internal.project_loader._config as _config
from tests import unit


class GetMetadataTest(ProjectLoaderBaseTest):

    def test_get_metadata(self):
        project_config = self.make_snapcraft_project(dedent("""\
            name: test
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
        """))
        metadata = project_config.get_metadata()

        self.assertThat(metadata['name'], Equals('test'))
        self.assertThat(metadata['version'], Equals('1'))
        self.assertThat(metadata['arch'], Equals(['amd64']))

    def test_get_metadata_version_adopted(self):
        project_config = self.make_snapcraft_project(dedent("""\
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
        metadata = project_config.get_metadata()

        self.assertThat(metadata['name'], Equals('test'))
        self.assertThat(metadata['version'], Is(None))
        self.assertThat(metadata['arch'], Equals(['amd64']))


class VariableExpansionTest(LoadPartBaseTest):

    def test_version_script(self):
        project_config = self.make_snapcraft_project(dedent("""\
            name: test
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
        """))  # noqa: E501

        self.assertThat(project_config.data['version-script'],
                        Equals('echo 1.0-devel'))

    def test_config_expands_filesets(self):
        self.make_snapcraft_project(dedent("""\
            name: test
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
        """))

        self.mock_load_part.assert_called_with('part1', 'go', {
            'prime': ['/usr/lib/wget.so', '/usr/bin/wget',
                      '/usr/share/my-icon.png'],
            'plugin': 'go', 'stage-packages': ['fswebcam'],
            'stage': ['/usr/lib/wget.so', '/usr/bin/wget', '/usr/lib/wget.a'],
        })

    def test_replace_snapcraft_variables(self):
        self.make_snapcraft_project(dedent("""\
            name: project-name
            version: "1"
            summary: test
            description: test
            confinement: strict

            parts:
              main:
                plugin: make
                source: $SNAPCRAFT_PROJECT_NAME-$SNAPCRAFT_PROJECT_VERSION
                make-options: [DEP=$SNAPCRAFT_STAGE]
            """))

        self.mock_load_part.assert_called_with('main', 'make', {
            'source': 'project-name-1',
            'plugin': 'make', 'stage': [], 'prime': [],
            'make-options': ['DEP={}'.format(self.stage_dir)],
        })


class DependenciesTest(ProjectLoaderBaseTest):

    def test_get_prereqs(self):
        project_config = self.make_snapcraft_project(dedent("""\
            name: test
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
        """))

        self.assertFalse(project_config.parts.get_prereqs('main'))
        self.assertThat(project_config.parts.get_prereqs('dependent'),
                        Equals({'main'}))

    def test_get_dependents(self):
        project_config = self.make_snapcraft_project(dedent("""\
            name: test
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
        """))

        self.assertFalse(project_config.parts.get_dependents('dependent'))
        self.assertThat(project_config.parts.get_dependents('main'),
                        Equals({'dependent'}))

    def test_dependency_loop(self):
        snapcraft_yaml = dedent("""\
            name: test
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
            self.make_snapcraft_project, snapcraft_yaml)

        self.assertThat(
            raised.message,
            Equals('circular dependency chain found in parts definition'))


class FilesetsTest(unit.TestCase):

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


class VariableReplacementsTest(unit.TestCase):

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
                subject, {
                    '$SNAPCRAFT_PROJECT_NAME': 'project_name',
                    '$SNAPCRAFT_PROJECT_VERSION': 'version',
                    '$SNAPCRAFT_STAGE': self.stage_dir,
                }), Equals(expected))

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
                subject, {
                    '$SNAPCRAFT_PROJECT_NAME': 'project_name',
                    '$SNAPCRAFT_PROJECT_VERSION': 'version',
                    '$SNAPCRAFT_STAGE': self.stage_dir,
                }), Equals(expected))

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
                subject, {
                    '$SNAPCRAFT_PROJECT_NAME': 'project_name',
                    '$SNAPCRAFT_PROJECT_VERSION': 'version',
                    '$SNAPCRAFT_STAGE': self.stage_dir,
                }), Equals(expected))

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
                subject, {
                    '$SNAPCRAFT_PROJECT_NAME': 'project_name',
                    '$SNAPCRAFT_PROJECT_VERSION': 'version',
                    '$SNAPCRAFT_STAGE': self.stage_dir,
                }), Equals(expected))

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
            subject, {
                '$SNAPCRAFT_PROJECT_NAME': 'project_name',
                '$SNAPCRAFT_PROJECT_VERSION': 'version',
                '$SNAPCRAFT_STAGE': self.stage_dir,
            }), Equals(expected))
