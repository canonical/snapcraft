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

import codecs
import contextlib
import logging
import os
import os.path
import sys

import jsonschema
import yaml

from snapcraft import (
    common,
    libraries,
    pluginhandler,
    sources,
    wiki,
)


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


class SnapcraftSchemaError(Exception):

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

    def __init__(self):
        self.build_tools = []
        self.all_parts = []
        self._part_names = []
        self.after_requests = {}

        self.data = _snapcraft_yaml_load()

        # To make the transition less painful
        self._remap_skills_to_interfaces()

        _validate_snapcraft_yaml(self.data)

        self.build_tools = self.data.get('build-packages', [])

        self._wiki = wiki.Wiki()

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
            self.data['architectures'] = [common.get_arch(), ]

    def _remap_skills_to_interfaces(self):
        if 'uses' in self.data:
            logger.warning(
                "DEPRECATED: Instances of 'uses' remapped to 'plugs'")
            self.data['plugs'] = self.data['uses']
            del self.data['uses']

        for slot in self.data.get('plugs', []):
            if 'type' in self.data['plugs'][slot]:
                slot_interface = self.data['plugs'][slot]['type']
                if slot_interface == 'migration-skill':
                    slot_interface = 'old-security'
                self.data['plugs'][slot]['interface'] = slot_interface
                del self.data['plugs'][slot]['type']
        for app in self.data.get('apps', []):
            if 'uses' in self.data['apps'][app]:
                self.data['apps'][app]['plugs'] = \
                    self.data['apps'][app]['uses']
                del self.data['apps'][app]['uses']

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

    def validate_parts(self, part_names):
        for part_name in part_names:
            if part_name not in self._part_names:
                raise EnvironmentError(
                    'The part named {!r} is not defined in '
                    '\'snapcraft.yaml\''.format(part_name))

    def load_plugin(self, part_name, plugin_name, properties):
        part = pluginhandler.load_plugin(
            part_name, plugin_name, properties)

        self.build_tools += part.code.build_packages
        self.build_tools += sources.get_required_packages(part.code.options)
        self.all_parts.append(part)
        return part

    def build_env_for_part(self, part, root_part=True):
        """Return a build env of all the part's dependencies."""

        env = []
        stagedir = common.get_stagedir()
        for dep_part in part.deps:
            env += dep_part.env(stagedir)
            env += self.build_env_for_part(dep_part, root_part=False)

        if root_part:
            env += part.env(part.installdir)
            env += _runtime_env(stagedir)
            env += _runtime_env(part.installdir)
            env += _build_env_for_stage(stagedir)
        else:
            env += part.env(stagedir)
            env += _runtime_env(stagedir)

        return env

    def stage_env(self):
        stagedir = common.get_stagedir()
        env = []

        env += _runtime_env(stagedir)
        env += _build_env_for_stage(stagedir)
        for part in self.all_parts:
            env += part.env(stagedir)

        return env

    def snap_env(self):
        snapdir = common.get_snapdir()
        env = []

        env += _runtime_env(snapdir)
        for part in self.all_parts:
            env += part.env(snapdir)

        return env


def _runtime_env(root):
    """Set the environment variables required for running binaries."""
    env = []

    env.append('PATH="' + ':'.join([
        '{0}/bin',
        '{0}/usr/bin',
        '$PATH'
    ]).format(root) + '"')

    # Add the default LD_LIBRARY_PATH
    env.append('LD_LIBRARY_PATH="' + ':'.join([
        '{0}/lib',
        '{0}/usr/lib',
        '{0}/lib/{1}',
        '{0}/usr/lib/{1}',
        '$LD_LIBRARY_PATH'
    ]).format(root, common.get_arch_triplet()) + '"')

    # Add more specific LD_LIBRARY_PATH if necessary
    ld_library_paths = libraries.determine_ld_library_path(root)
    if ld_library_paths:
        env.append('LD_LIBRARY_PATH="' + ':'.join(ld_library_paths) +
                   ':$LD_LIBRARY_PATH"')

    return env


def _build_env(root):
    """Set the environment variables required for building.

    This is required for the current parts installdir due to stage-packages
    and also to setup the stagedir.
    """
    env = []

    arch_triplet = common.get_arch_triplet()

    env.append('CFLAGS="' + ' '.join([
        '-I{0}/include',
        '-I{0}/usr/include',
        '-I{0}/include/{1}',
        '-I{0}/usr/include/{1}',
        '$CFLAGS'
    ]).format(root, arch_triplet) + '"')
    env.append('CXXFLAGS="' + ' '.join([
        '-I{0}/include',
        '-I{0}/usr/include',
        '-I{0}/include/{1}',
        '-I{0}/usr/include/{1}',
        '$CFLAGS'
    ]).format(root, arch_triplet) + '"')
    env.append('CPPFLAGS="' + ' '.join([
        '-I{0}/include',
        '-I{0}/usr/include',
        '-I{0}/include/{1}',
        '-I{0}/usr/include/{1}',
        '$CPPFLAGS'
    ]).format(root, arch_triplet) + '"')
    env.append('LDFLAGS="' + ' '.join([
        '-L{0}/lib',
        '-L{0}/usr/lib',
        '-L{0}/lib/{1}',
        '-L{0}/usr/lib/{1}',
        '$LDFLAGS'
    ]).format(root, arch_triplet) + '"')
    env.append('PKG_CONFIG_PATH=' + ':'.join([
        '{0}/lib/pkgconfig',
        '{0}/lib/{1}/pkgconfig',
        '{0}/usr/lib/pkgconfig',
        '{0}/usr/lib/{1}/pkgconfig',
        '{0}/usr/share/pkgconfig',
        '{0}/usr/local/lib/pkgconfig',
        '{0}/usr/local/lib/{1}/pkgconfig',
        '{0}/usr/local/share/pkgconfig',
        '$PKG_CONFIG_PATH'
    ]).format(root, arch_triplet))

    return env


def _build_env_for_stage(stagedir):
    env = _build_env(stagedir)

    env.append('PERL5LIB={0}/usr/share/perl5/'.format(stagedir))
    env.append('PKG_CONFIG_SYSROOT_DIR={0}'.format(stagedir))

    return env


def _validate_snapcraft_yaml(snapcraft_yaml):
    schema_file = os.path.abspath(os.path.join(common.get_schemadir(),
                                               'snapcraft.yaml'))

    try:
        with open(schema_file) as fp:
            schema = yaml.load(fp)
            format_check = jsonschema.FormatChecker()
            jsonschema.validate(snapcraft_yaml, schema,
                                format_checker=format_check)
    except FileNotFoundError:
        raise SnapcraftSchemaError(
            'snapcraft validation file is missing from installation path')
    except jsonschema.ValidationError as e:
        messages = [e.message]
        if e.path:
            messages.insert(0, "The '{}' property does not match the "
                               "required schema:".format(e.path.pop()))

        raise SnapcraftSchemaError(' '.join(messages))


def _snapcraft_yaml_load(yaml_file='snapcraft.yaml'):
    try:
        with open(yaml_file, 'rb') as fp:
            bs = fp.read(2)
    except FileNotFoundError:
        raise SnapcraftYamlFileError(yaml_file)

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


def _expand_filesets_for(stage, properties):
    filesets = properties.get('filesets', {})
    fileset_for_stage = properties.get(stage, {})
    new_stage_set = []

    for item in fileset_for_stage:
        if item.startswith('$'):
            try:
                new_stage_set.extend(filesets[item[1:]])
            except KeyError:
                raise SnapcraftLogicError(
                    '\'{}\' referred to in the \'{}\' fileset but it is not '
                    'in filesets'.format(item, stage))
        else:
            new_stage_set.append(item)

    return new_stage_set


def load_config():
    try:
        return Config()
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
