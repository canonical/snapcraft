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
import contextlib
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
    pluginhandler,
    sources,
    wiki,
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


class SnapcraftLogicError(Exception):

    @property
    def message(self):
        return self._message

    def __init__(self, message):
        self._message = message


class PluginNotDefinedError(Exception):

    @property
    def part(self):
        return self._part

    def __init__(self, part):
        self._part = part


class Config:

    @property
    def part_names(self):
        return self._part_names

    def __init__(self, project_options=None):
        if project_options is None:
            project_options = snapcraft.ProjectOptions()

        self.build_tools = []
        self.all_parts = []
        self._part_names = []
        self._project_options = project_options
        self.after_requests = {}

        self._snapcraft_yaml = _get_snapcraft_yaml()
        self.data = _snapcraft_yaml_load(self._snapcraft_yaml)

        self._validator = Validator(self.data)
        self._validator.validate()
        _ensure_confinement_default(self.data, self._validator.schema)

        self.build_tools = self.data.get('build-packages', [])
        self.build_tools.extend(project_options.additional_build_packages)

        self._wiki = wiki.Wiki()
        self._process_parts()

    def _process_parts(self):
        for part_name in self.data.get('parts', []):
            self._part_names.append(part_name)
            properties = self.data['parts'][part_name] or {}

            plugin_name = properties.pop('plugin', None)
            if not plugin_name:
                logger.info(
                    'Searching the wiki to compose part "{}"'.format(
                        part_name))
                with contextlib.suppress(KeyError):
                    properties = self._wiki.compose(part_name, properties)
                    plugin_name = properties.pop('plugin', None)
                    # The wiki still supports using 'type' for snapcraft 1.x
                    if 'type' in properties:
                        del properties['type']

            if not plugin_name:
                raise PluginNotDefinedError(part_name)

            if 'after' in properties:
                self.after_requests[part_name] = properties.pop('after')

            properties['stage'] = _expand_filesets_for('stage', properties)
            properties['snap'] = _expand_filesets_for('snap', properties)

            if 'filesets' in properties:
                del properties['filesets']

            self.load_plugin(part_name, plugin_name, properties)

        self._compute_part_dependencies()
        self.all_parts = self._sort_parts()

        if 'architectures' not in self.data:
            self.data['architectures'] = [self._project_options.deb_arch]

    def _compute_part_dependencies(self):
        '''Gather the lists of dependencies and adds to all_parts.'''

        for part in self.all_parts:
            dep_names = self.after_requests.get(part.name, [])
            for dep in dep_names:
                found = False
                for i in range(len(self.all_parts)):
                    if dep == self.all_parts[i].name:
                        part.deps.append(self.all_parts[i])
                        found = True
                        break
                if not found:
                    wiki_part = self._wiki.get_part(dep)
                    found = True if wiki_part else False
                    if found:
                        plugin_name = wiki_part.pop('plugin')
                        part.deps.append(self.load_plugin(
                            dep, plugin_name, wiki_part))
                        self._part_names.append(dep)
                if not found:
                    raise SnapcraftLogicError(
                        'part name missing {}'.format(dep))

    def _sort_parts(self):
        '''Performs an inneficient but easy to follow sorting of parts.'''
        sorted_parts = []

        while self.all_parts:
            top_part = None
            for part in self.all_parts:
                mentioned = False
                for other in self.all_parts:
                    if part in other.deps:
                        mentioned = True
                        break
                if not mentioned:
                    top_part = part
                    break
            if not top_part:
                raise SnapcraftLogicError(
                    'circular dependency chain found in parts definition')
            sorted_parts = [top_part] + sorted_parts
            self.all_parts.remove(top_part)

        return sorted_parts

    def part_prereqs(self, part_name):
        """Returns a set with all of part_names' prerequisites."""
        return set(self.after_requests.get(part_name, []))

    def part_dependents(self, part_name):
        """Returns a set of all the parts that depend upon part_name."""

        dependents = set()
        for part, prerequisites in self.after_requests.items():
            if part_name in prerequisites:
                dependents.add(part)

        return dependents

    def get_part(self, part_name):
        for part in self.all_parts:
            if part.name == part_name:
                return part

        return None

    def get_project_state(self, step):
        """Returns a dict of states for the given step of each part."""

        state = {}
        for part in self.all_parts:
            state[part.name] = part.get_state(step)

        return state

    def validate_parts(self, part_names):
        for part_name in part_names:
            if part_name not in self._part_names:
                raise EnvironmentError(
                    'The part named {!r} is not defined in '
                    '{!r}'.format(part_name, self._snapcraft_yaml))

    def load_plugin(self, part_name, plugin_name, properties):
        part = pluginhandler.load_plugin(
            part_name, plugin_name, properties,
            self._project_options, self._validator.part_schema)

        self.build_tools += part.code.build_packages
        self.build_tools += sources.get_required_packages(part.code.options)
        self.all_parts.append(part)
        return part

    def build_env_for_part(self, part, root_part=True):
        """Return a build env of all the part's dependencies."""

        env = []
        stagedir = self._project_options.stage_dir

        if root_part:
            # this has to come before any {}/usr/bin
            env += _create_pkg_config_override(
                part.bindir, part.installdir, stagedir,
                self._project_options.arch_triplet)
            env += part.env(part.installdir)
            env += _runtime_env(part.installdir,
                                self._project_options.arch_triplet)
            env += _runtime_env(stagedir,
                                self._project_options.arch_triplet)
            env += _build_env(part.installdir,
                              self._project_options.arch_triplet)
            env += _build_env_for_stage(stagedir,
                                        self._project_options.arch_triplet)
        else:
            env += part.env(stagedir)
            env += _runtime_env(stagedir,
                                self._project_options.arch_triplet)

        for dep_part in part.deps:
            env += dep_part.env(stagedir)
            env += self.build_env_for_part(dep_part, root_part=False)

        return env

    def stage_env(self):
        stage_dir = self._project_options.stage_dir
        env = []

        env += _runtime_env(stage_dir, self._project_options.arch_triplet)
        env += _build_env_for_stage(stage_dir,
                                    self._project_options.arch_triplet)
        for part in self.all_parts:
            env += part.env(stage_dir)

        return env

    def snap_env(self):
        snap_dir = self._project_options.snap_dir
        env = []

        env += _runtime_env(snap_dir, self._project_options.arch_triplet)
        dependency_paths = set()
        for part in self.all_parts:
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


def _runtime_env(root, arch_triplet):
    """Set the environment variables required for running binaries."""
    env = []

    env.append('PATH="' + ':'.join([
        '{0}/bin',
        '{0}/usr/bin',
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

    return env


def _build_env_for_stage(stagedir, arch_triplet):
    env = _build_env(stagedir, arch_triplet)
    env.append('PERL5LIB={0}/usr/share/perl5/'.format(stagedir))

    return env


_PKG_CONFIG_TEMPLATE = """#!/usr/bin/env python3

import os
import sys
from subprocess import check_call, CalledProcessError

real_env = os.environ.copy()


def run_pkg_config(args, env):
    check_call(['/usr/bin/pkg-config'] + args, env=env)


def get_pkg_env_for(basedir):
    current_env = real_env.copy()
    env = {{}}
    env['PKG_CONFIG_PATH'] = ':'.join([
        '{{basedir}}/lib/pkgconfig',
        '{{basedir}}/lib/{arch_triplet}/pkgconfig',
        '{{basedir}}/usr/lib/pkgconfig',
        '{{basedir}}/usr/lib/{arch_triplet}/pkgconfig',
        '{{basedir}}/usr/share/pkgconfig',
        '{{basedir}}/usr/local/lib/pkgconfig',
        '{{basedir}}/usr/local/lib/{arch_triplet}/pkgconfig',
        '{{basedir}}/usr/local/share/pkgconfig']).format(basedir=basedir)
    env['PKG_CONFIG_SYSROOT_DIR'] = basedir
    env['PKG_CONFIG_LIBDIR'] = ''

    current_env.update(env)

    return current_env


def modules_exist(modules, env):
    try:
        check_call(['/usr/bin/pkg-config', '--exists'] + modules, env=env)
        return True
    except CalledProcessError:
        return False


def main():
    args = sys.argv[1:]
    modules = list(filter(lambda x: not x.startswith('-'), args))

    env = get_pkg_env_for('{installdir}')
    if modules_exist(modules, env):
        run_pkg_config(args, env)
        return

    env = get_pkg_env_for('{stagedir}')
    if modules_exist(modules, env):
        run_pkg_config(args, env)
        return

    run_pkg_config(args, env=real_env)


if __name__ == '__main__':
    main()
"""


def _create_pkg_config_override(bindir, installdir, stagedir, arch_triplet):
    pkg_config_path = os.path.join(bindir, 'pkg-config')
    os.makedirs(os.path.dirname(pkg_config_path), exist_ok=True)

    pkg_config_content = _PKG_CONFIG_TEMPLATE.format(
        installdir=installdir, stagedir=stagedir, arch_triplet=arch_triplet)

    with open(pkg_config_path, 'w') as fn:
        fn.write(pkg_config_content)
    os.chmod(pkg_config_path, 0o755)

    return ['PATH={}:$PATH'.format(bindir)]


def _expand_filesets_for(step, properties):
    filesets = properties.get('filesets', {})
    fileset_for_step = properties.get(step, {})
    new_step_set = []

    for item in fileset_for_step:
        if item.startswith('$'):
            try:
                new_step_set.extend(filesets[item[1:]])
            except KeyError:
                raise SnapcraftLogicError(
                    '\'{}\' referred to in the \'{}\' fileset but it is not '
                    'in filesets'.format(item, step))
        else:
            new_step_set.append(item)

    return new_step_set


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
    except SnapcraftLogicError as e:
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
