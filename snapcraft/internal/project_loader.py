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

import codecs
import logging
import os
import os.path
import re
import sys

import jsonschema
import yaml

import snapcraft
from snapcraft.internal import (
    common,
    libraries,
    parts,
    pluginhandler,
)
from snapcraft._schema import Validator, SnapcraftSchemaError


logger = logging.getLogger(__name__)


@jsonschema.FormatChecker.cls_checks('file-path')
def _validate_file_exists(instance):
    if not os.path.exists(instance):
        raise jsonschema.exceptions.ValidationError(
            "Specified file '{}' does not exist".format(instance))

    return True


@jsonschema.FormatChecker.cls_checks('icon-path')
def _validate_icon(instance):
    allowed_extensions = ['.png', '.svg']
    extension = os.path.splitext(instance.lower())[1]
    if extension not in allowed_extensions:
        raise jsonschema.exceptions.ValidationError(
            "'icon' must be either a .png or a .svg")

    if not os.path.exists(instance):
        raise jsonschema.exceptions.ValidationError(
            "Specified icon '{}' does not exist".format(instance))

    return True


class InvalidEpochError(Exception):
    pass


@jsonschema.FormatChecker.cls_checks('epoch', raises=InvalidEpochError)
def _validate_epoch(instance):
    str_instance = str(instance)
    pattern = re.compile('^(?:0|[1-9][0-9]*[*]?)$')
    if not pattern.match(str_instance):
        raise InvalidEpochError(
            "epochs are positive integers followed by an optional asterisk")

    return True


class SnapcraftYamlFileError(Exception):

    @property
    def file(self):
        return self._file

    def __init__(self, yaml_file):
        self._file = yaml_file


class PluginNotDefinedError(Exception):

    @property
    def part(self):
        return self._part

    def __init__(self, part):
        self._part = part


class Config:

    @property
    def part_names(self):
        return self.parts.part_names

    @property
    def all_parts(self):
        return self.parts.all_parts

    def __init__(self, project_options=None):
        if project_options is None:
            project_options = snapcraft.ProjectOptions()

        self.build_tools = []
        self._project_options = project_options

        self._snapcraft_yaml = _get_snapcraft_yaml()
        snapcraft_yaml = _snapcraft_yaml_load(self._snapcraft_yaml)
        self._validator = Validator(snapcraft_yaml)
        self._validator.validate()
        self.data = self._expand_env(snapcraft_yaml)

        # both confinement type and build quality are optionals
        _ensure_confinement_default(self.data, self._validator.schema)
        _ensure_grade_default(self.data, self._validator.schema)

        self.build_tools = self.data.get('build-packages', [])
        self.build_tools.extend(project_options.additional_build_packages)

        self.parts = parts.PartsConfig(self.data.get('parts', {}),
                                       self._project_options,
                                       self._validator,
                                       self.build_tools,
                                       self._snapcraft_yaml)

        if 'architectures' not in self.data:
            self.data['architectures'] = [self._project_options.deb_arch]

    def get_project_state(self, step):
        """Returns a dict of states for the given step of each part."""

        state = {}
        for part in self.parts.all_parts:
            state[part.name] = part.get_state(step)

        return state

    def stage_env(self):
        stage_dir = self._project_options.stage_dir
        env = []

        env += _runtime_env(stage_dir, self._project_options.arch_triplet)
        env += _build_env_for_stage(stage_dir,
                                    self._project_options.arch_triplet)
        for part in self.parts.all_parts:
            env += part.env(stage_dir)

        return env

    def snap_env(self):
        snap_dir = self._project_options.snap_dir
        env = []

        env += _runtime_env(snap_dir, self._project_options.arch_triplet)
        dependency_paths = set()
        for part in self.parts.all_parts:
            env += part.env(snap_dir)
            dependency_paths |= part.get_primed_dependency_paths()

        # Dependency paths are only valid if they actually exist. Sorting them
        # here as well so the LD_LIBRARY_PATH is consistent between runs.
        dependency_paths = sorted({
            path for path in dependency_paths if os.path.isdir(path)})

        if dependency_paths:
            # Add more specific LD_LIBRARY_PATH from the dependencies.
            env.append('LD_LIBRARY_PATH="' + ':'.join(dependency_paths) +
                       ':$LD_LIBRARY_PATH"')

        return env

    def project_env(self):
        return [
            'SNAPCRAFT_STAGE={}'.format(self._project_options.stage_dir),
            'SNAPCRAFT_PROJECT_NAME={}'.format(self.data['name']),
            'SNAPCRAFT_PROJECT_VERSION={}'.format(self.data['version']),
        ]

    def _expand_env(self, snapcraft_yaml):
        environment_keys = ['name', 'version']
        for key in snapcraft_yaml:
            if any((key == env_key for env_key in environment_keys)):
                continue
            snapcraft_yaml[key] = _replace_attr(
                snapcraft_yaml[key],
                snapcraft_yaml['name'],
                snapcraft_yaml['version'],
                self._project_options.stage_dir)
        return snapcraft_yaml


def _replace_attr(attr, name, version, stage_dir):
    if isinstance(attr, str):
        attr = attr.replace('$SNAPCRAFT_STAGE', stage_dir)
        attr = attr.replace('$SNAPCRAFT_PROJECT_NAME', name)
        attr = attr.replace('$SNAPCRAFT_PROJECT_VERSION', str(version))
        return attr
    elif isinstance(attr, list) or isinstance(attr, tuple):
        return [_replace_attr(i, name, version, stage_dir)
                for i in attr]
    elif isinstance(attr, dict):
        return {k: _replace_attr(attr[k], name, version, stage_dir)
                for k in attr}

    return attr


def _runtime_env(root, arch_triplet):
    """Set the environment variables required for running binaries."""
    env = []

    env.append('PATH="' + ':'.join([
        '{0}/usr/sbin',
        '{0}/usr/bin',
        '{0}/sbin',
        '{0}/bin',
        '$PATH'
    ]).format(root) + '"')

    # Add the default LD_LIBRARY_PATH
    paths = common.get_library_paths(root, arch_triplet)
    if paths:
        env.append(common.format_path_variable(
            'LD_LIBRARY_PATH', paths, prepend='', separator=':'))

    # Add more specific LD_LIBRARY_PATH from staged packages if necessary
    ld_library_paths = libraries.determine_ld_library_path(root)
    if ld_library_paths:
        env.append('LD_LIBRARY_PATH="' + ':'.join(ld_library_paths) +
                   ':$LD_LIBRARY_PATH"')

    return env


def _build_env(root, arch_triplet):
    """Set the environment variables required for building.

    This is required for the current parts installdir due to stage-packages
    and also to setup the stagedir.
    """
    env = []

    paths = common.get_include_paths(root, arch_triplet)
    if paths:
        for envvar in ['CPPFLAGS', 'CFLAGS', 'CXXFLAGS']:
            env.append(common.format_path_variable(
                envvar, paths, prepend='-I', separator=' '))
    paths = common.get_library_paths(root, arch_triplet)
    if paths:
        env.append(common.format_path_variable(
            'LDFLAGS', paths, prepend='-L', separator=' '))
    paths = common.get_pkg_config_paths(root, arch_triplet)
    if paths:
        env.append(common.format_path_variable(
            'PKG_CONFIG_PATH', paths, prepend='', separator=':'))

    return env


def _build_env_for_stage(stagedir, arch_triplet):
    env = _build_env(stagedir, arch_triplet)
    env.append('PERL5LIB={0}/usr/share/perl5/'.format(stagedir))

    return env


def _get_snapcraft_yaml():
    visible_yaml_exists = os.path.exists('snapcraft.yaml')
    hidden_yaml_exists = os.path.exists('.snapcraft.yaml')

    if visible_yaml_exists and hidden_yaml_exists:
        raise EnvironmentError(
            "Found a 'snapcraft.yaml' and a '.snapcraft.yaml', "
            "please remove one")
    elif visible_yaml_exists:
        return 'snapcraft.yaml'
    elif hidden_yaml_exists:
        return '.snapcraft.yaml'
    else:
        raise SnapcraftYamlFileError('snapcraft.yaml')


def _snapcraft_yaml_load(yaml_file):
    with open(yaml_file, 'rb') as fp:
        bs = fp.read(2)

    if bs == codecs.BOM_UTF16_LE or bs == codecs.BOM_UTF16_BE:
        encoding = 'utf-16'
    else:
        encoding = 'utf-8'

    try:
        with open(yaml_file, encoding=encoding) as fp:
            return yaml.load(fp)
    except yaml.scanner.ScannerError as e:
        raise SnapcraftSchemaError(
            '{} on line {} of {}'.format(
                e.problem, e.problem_mark.line, yaml_file))


def load_config(project_options=None):
    try:
        return Config(project_options)
    except SnapcraftYamlFileError as e:
        logger.error(
            'Could not find {}.  Are you sure you are in the right '
            'directory?\nTo start a new project, use \'snapcraft '
            'init\''.format(e.file))
        sys.exit(1)
    except SnapcraftSchemaError as e:
        msg = 'Issues while validating snapcraft.yaml: {}'.format(e.message)
        logger.error(msg)
        sys.exit(1)
    except PluginNotDefinedError as e:
        logger.error(
            'Issues while validating snapcraft.yaml: the "plugin" keyword is '
            'missing for the "{}" part.'.format(e.part))
        sys.exit(1)
    except parts.SnapcraftLogicError as e:
        logger.error('Issue detected while analyzing '
                     'snapcraft.yaml: {}'.format(e.message))
        sys.exit(1)
    except pluginhandler.PluginError as e:
        logger.error('Issue while loading plugin: {}'.format(e))
        sys.exit(1)


def _ensure_confinement_default(yaml_data, schema):
    # Provide hint if the confinement property is missing, and add the
    # default. We use the schema here so we don't have to hard-code defaults.
    if 'confinement' not in yaml_data:
        logger.warning('"confinement" property not specified: defaulting '
                       'to "strict"')
        yaml_data['confinement'] = schema['confinement']['default']


def _ensure_grade_default(yaml_data, schema):
    # Provide hint if the grade property is missing, and add the
    # default. We use the schema here so we don't have to hard-code defaults.
    if 'grade' not in yaml_data:
        logger.warning('"grade" property not specified: defaulting '
                       'to "stable"')
        yaml_data['grade'] = schema['grade']['default']
