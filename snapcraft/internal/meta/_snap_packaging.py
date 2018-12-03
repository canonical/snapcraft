# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

import copy
import collections
import contextlib
import itertools
import logging
import os
import re
import shlex
import shutil
import stat
import subprocess
from typing import Any, Dict, List, Set  # noqa

from snapcraft import file_utils, formatting_utils, yaml_utils
from snapcraft import shell_utils
from snapcraft.project import _schema
from snapcraft.internal import common, errors, project_loader
from snapcraft.internal.project_loader import _config
from snapcraft.extractors import _metadata
from snapcraft.internal.deprecations import handle_deprecation_notice
from snapcraft.internal.meta import (
    _desktop,
    _errors as meta_errors,
    _manifest,
    _version,
)


logger = logging.getLogger(__name__)


_MANDATORY_PACKAGE_KEYS = ["name", "version", "summary", "description"]

_OPTIONAL_PACKAGE_KEYS = [
    "architectures",
    "assumes",
    "base",
    "confinement",
    "environment",
    "epoch",
    "grade",
    "hooks",
    "layout",
    "license",
    "plugs",
    "slots",
    "title",
    "type",
]


class OctInt(yaml_utils.SnapcraftYAMLObject):
    """An int represented in octal form."""

    yaml_tag = u"!OctInt"

    def __init__(self, value):
        super().__init__()
        self._value = value

    @classmethod
    def to_yaml(cls, dumper, data):
        """
        Convert a Python object to a representation node.
        """
        return dumper.represent_scalar(
            "tag:yaml.org,2002:int", "{:04o}".format(data._value)
        )


# From snapd's snap/validate.go, appContentWhitelist, with a slight modification: don't
# allow leading slashes.
_APP_COMMAND_PATTERN = re.compile("^[A-Za-z0-9. _#:$-][A-Za-z0-9/. _#:$-]*$")


def create_snap_packaging(project_config: _config.Config) -> str:
    """Create snap.yaml and related assets in meta.

    Create the meta directory and provision it with snap.yaml in the snap dir
    using information from config_data. Also copy in the local 'snap'
    directory, and generate wrappers for hooks coming from parts.

    :param dict config_data: project values defined in snapcraft.yaml.
    :return: meta_dir.
    """

    # Update config_data using metadata extracted from the project
    _update_yaml_with_extracted_metadata(project_config.data, project_config.parts)

    # Now that we've updated config_data with random stuff extracted from
    # parts, re-validate it to ensure the it still conforms with the schema.
    validator = _schema.Validator(project_config.data)
    validator.validate(source="properties")

    # Update default values
    _update_yaml_with_defaults(project_config.data, project_config.validator.schema)

    # Ensure the YAML contains all required keywords before continuing to
    # use it to generate the snap.yaml.
    _ensure_required_keywords(project_config.data)

    packaging = _SnapPackaging(project_config)
    packaging.write_snap_yaml()
    packaging.setup_assets()
    packaging.generate_hook_wrappers()
    packaging.write_snap_directory()

    return packaging.meta_dir


def _update_yaml_with_extracted_metadata(
    config_data: Dict[str, Any], parts_config: project_loader.PartsConfig
) -> None:
    if "adopt-info" in config_data:
        part_name = config_data["adopt-info"]
        part = parts_config.get_part(part_name)
        if not part:
            raise meta_errors.AdoptedPartMissingError(part_name)

        pull_state = part.get_pull_state()
        build_state = part.get_build_state()
        stage_state = part.get_stage_state()
        prime_state = part.get_prime_state()

        # Get the metadata from the pull step first.
        metadata = pull_state.extracted_metadata["metadata"]

        # Now update it using the metadata from the build step (i.e. the data
        # from the build step takes precedence over the pull step).
        metadata.update(build_state.extracted_metadata["metadata"])

        # Now make sure any scriptlet data are taken into account. Later steps
        # take precedence, and scriptlet data (even in earlier steps) take
        # precedence over extracted data.
        metadata.update(pull_state.scriptlet_metadata)
        metadata.update(build_state.scriptlet_metadata)
        metadata.update(stage_state.scriptlet_metadata)
        metadata.update(prime_state.scriptlet_metadata)

        if not metadata:
            # If we didn't end up with any metadata, let's ensure this part was
            # actually supposed to parse info. If not, let's try to be very
            # clear about what's happening, here. We do this after checking for
            # metadata because metadata could be supplied by scriptlets, too.
            if "parse-info" not in config_data["parts"][part_name]:
                raise meta_errors.AdoptedPartNotParsingInfo(part_name)

        _adopt_info(config_data, metadata)


def _adopt_info(
    config_data: Dict[str, Any], extracted_metadata: _metadata.ExtractedMetadata
):
    ignored_keys = _adopt_keys(config_data, extracted_metadata)
    if ignored_keys:
        logger.warning(
            "The {keys} {plural_property} {plural_is} specified in adopted "
            "info as well as the YAML: taking the {plural_property} from the "
            "YAML".format(
                keys=formatting_utils.humanize_list(list(ignored_keys), "and"),
                plural_property=formatting_utils.pluralize(
                    ignored_keys, "property", "properties"
                ),
                plural_is=formatting_utils.pluralize(ignored_keys, "is", "are"),
            )
        )


def _adopt_keys(
    config_data: Dict[str, Any], extracted_metadata: _metadata.ExtractedMetadata
) -> Set[str]:
    ignored_keys = set()
    metadata_dict = extracted_metadata.to_dict()

    # desktop_file_paths and common_ids are special cases that will be handled
    # after all the top level snapcraft.yaml keys.
    ignore = ("desktop_file_paths", "common_id")
    overrides = ((k, v) for k, v in metadata_dict.items() if k not in ignore)

    for key, value in overrides:
        if key not in config_data:
            if key == "icon":
                if _icon_file_exists() or not os.path.exists(str(value)):
                    # Do not overwrite the icon file.
                    continue
            config_data[key] = value
        else:
            ignored_keys.add(key)

    if "desktop_file_paths" in metadata_dict and "common_id" in metadata_dict:
        app_name = _get_app_name_from_common_id(
            config_data, str(metadata_dict["common_id"])
        )
        if app_name and not _desktop_file_exists(app_name):
            for desktop_file_path in metadata_dict["desktop_file_paths"]:
                if os.path.exists(desktop_file_path):
                    config_data["apps"][app_name]["desktop"] = desktop_file_path
                    break

    return ignored_keys


def _icon_file_exists() -> bool:
    """Check if the icon is specified as a file in the assets dir.

    The icon file can be in two directories: 'setup/gui' (deprecated) or
    'snap/gui'. The icon can be named 'icon.png' or 'icon.svg'.

    :returns: True if the icon file exists, False otherwise.
    """
    for icon_path in (
        os.path.join(asset_dir, "gui", icon_file)
        for (asset_dir, icon_file) in itertools.product(
            ["setup", "snap"], ["icon.png", "icon.svg"]
        )
    ):
        if os.path.exists(icon_path):
            return True
    else:
        return False


def _get_app_name_from_common_id(config_data: Dict[str, Any], common_id: str) -> str:
    """Get the snap app name with a common-id.

    :params dict config_data: Project values defined in snapcraft.yaml.
    :params common_id: The common identifier across multiple packaging
        formats.
    :returns: The name of the snap app with the common-id.

    """
    if "apps" in config_data:
        for app in config_data["apps"]:
            if config_data["apps"][app].get("common-id") == common_id:
                return app
    return None


def _desktop_file_exists(app_name: str) -> bool:
    """Check if the desktop file is specified as a file in the assets dir.

    The desktop file can be in two directories: 'setup/gui' (deprecated) or
    'snap/gui'.

    :params app_name: The name of the snap app.
    :returns: True if the desktop file exists, False otherwise.
    """
    for desktop_path in (
        os.path.join(asset_dir, "gui", "{}.desktop".format(app_name))
        for asset_dir in ["setup", "snap"]
    ):
        if os.path.exists(desktop_path):
            return True
    else:
        return False


def _update_yaml_with_defaults(config_data, schema):
    # Ensure that grade and confinement have their defaults applied, if
    # necessary. Defaults are taken from the schema. Technically these are the
    # only two optional keywords currently WITH defaults, but we don't want to
    # risk setting something that we add later on accident.
    for key in ("confinement", "grade"):
        if key not in config_data:
            with contextlib.suppress(KeyError):
                default = schema[key]["default"]
                config_data[key] = default
                logger.warn(
                    "{!r} property not specified: defaulting to {!r}".format(
                        key, default
                    )
                )

    # Set default adapter
    app_schema = schema["apps"]["patternProperties"]["^[a-zA-Z0-9](?:-?[a-zA-Z0-9])*$"][
        "properties"
    ]
    default_adapter = app_schema["adapter"]["default"]
    for app in config_data.get("apps", {}).values():
        if "adapter" not in app:
            app["adapter"] = default_adapter


def _ensure_required_keywords(config_data):
    # Verify that all mandatory keys have been satisfied
    missing_keys = []  # type: List[str]
    for key in _MANDATORY_PACKAGE_KEYS:
        if key not in config_data:
            missing_keys.append(key)

    if missing_keys:
        raise meta_errors.MissingSnapcraftYamlKeysError(keys=missing_keys)


class _SnapPackaging:
    @property
    def meta_dir(self) -> str:
        return self._meta_dir

    def __init__(self, project_config: _config.Config) -> None:
        self._project_config = project_config
        self._snapcraft_yaml_path = project_config.project.info.snapcraft_yaml_file_path
        self._prime_dir = project_config.project.prime_dir
        self._parts_dir = project_config.project.parts_dir
        self._arch_triplet = project_config.project.arch_triplet
        self._global_state_file = project_config.project._global_state_file
        self._is_host_compatible_with_base = (
            project_config.project.is_host_compatible_with_base
        )
        self._meta_dir = os.path.join(self._prime_dir, "meta")
        self._config_data = project_config.data.copy()
        self._original_snapcraft_yaml = project_config.project.info.get_raw_snapcraft()
        self._meta_runner = os.path.join(
            self._prime_dir, "snap", "command-chain", "snapcraft-runner"
        )

        self._install_path_pattern = re.compile(
            r"{}/[a-z0-9][a-z0-9+-]*/install".format(re.escape(self._parts_dir))
        )

        os.makedirs(self._meta_dir, exist_ok=True)

    def write_snap_yaml(self) -> str:
        common.env = self._project_config.snap_env()

        try:
            # Only generate the meta command chain if there are apps that need it
            if self._config_data.get("apps", None):
                self._generate_command_chain()

            package_snap_path = os.path.join(self.meta_dir, "snap.yaml")
            snap_yaml = self._compose_snap_yaml()

            with open(package_snap_path, "w") as f:
                yaml_utils.dump(snap_yaml, stream=f)

            return snap_yaml
        finally:
            common.reset_env()

    def setup_assets(self) -> None:
        # We do _setup_from_setup first since it is legacy and let the
        # declarative items take over.
        self._setup_gui()

        if "icon" in self._config_data:
            # TODO: use developer.ubuntu.com once it has updated documentation.
            icon_ext = self._config_data["icon"].split(os.path.extsep)[-1]
            icon_dir = os.path.join(self.meta_dir, "gui")
            icon_path = os.path.join(icon_dir, "icon.{}".format(icon_ext))
            if not os.path.exists(icon_dir):
                os.mkdir(icon_dir)
            if os.path.exists(icon_path):
                os.unlink(icon_path)
            file_utils.link_or_copy(self._config_data["icon"], icon_path)

        if self._config_data.get("type", "") == "gadget":
            if not os.path.exists("gadget.yaml"):
                raise errors.MissingGadgetError()
            file_utils.link_or_copy(
                "gadget.yaml", os.path.join(self.meta_dir, "gadget.yaml")
            )

    def _generate_command_chain(self):
        # Classic confinement or building on a host that does not match the target base
        # means we cannot setup an environment that will work.
        if (
            self._config_data["confinement"] == "classic"
            or not self._is_host_compatible_with_base
        ):
            assembled_env = None
        else:
            assembled_env = common.assemble_env()
            assembled_env = assembled_env.replace(self._prime_dir, "$SNAP")
            assembled_env = self._install_path_pattern.sub("$SNAP", assembled_env)

            if assembled_env:
                os.makedirs(os.path.dirname(self._meta_runner), exist_ok=True)
                with open(self._meta_runner, "w") as f:
                    print("#!/bin/sh", file=f)
                    print(assembled_env, file=f)
                    print(
                        "export LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH",
                        file=f,
                    )
                    print('exec "$@"', file=f)
                os.chmod(self._meta_runner, 0o755)

    def _record_manifest_and_source_snapcraft_yaml(self):
        prime_snap_dir = os.path.join(self._prime_dir, "snap")
        recorded_snapcraft_yaml_path = os.path.join(prime_snap_dir, "snapcraft.yaml")
        if os.path.isfile(recorded_snapcraft_yaml_path):
            os.unlink(recorded_snapcraft_yaml_path)
        manifest_file_path = os.path.join(prime_snap_dir, "manifest.yaml")
        if os.path.isfile(manifest_file_path):
            os.unlink(manifest_file_path)

        # FIXME hide this functionality behind a feature flag for now
        if os.environ.get("SNAPCRAFT_BUILD_INFO"):
            os.makedirs(prime_snap_dir, exist_ok=True)
            shutil.copy2(self._snapcraft_yaml_path, recorded_snapcraft_yaml_path)
            annotated_snapcraft = _manifest.annotate_snapcraft(
                copy.deepcopy(self._config_data),
                self._parts_dir,
                self._global_state_file,
            )
            with open(manifest_file_path, "w") as manifest_file:
                yaml_utils.dump(annotated_snapcraft, stream=manifest_file)

    def write_snap_directory(self) -> None:
        # First migrate the snap directory. It will overwrite any conflicting
        # files.
        for root, directories, files in os.walk("snap"):
            with contextlib.suppress(ValueError):
                directories.remove(".snapcraft")
            with contextlib.suppress(ValueError):
                # The snapcraft.yaml is migrated later
                files.remove("snapcraft.yaml")

            for directory in directories:
                source = os.path.join(root, directory)
                destination = os.path.join(self._prime_dir, source)
                file_utils.create_similar_directory(source, destination)

            for file_path in files:
                source = os.path.join(root, file_path)
                destination = os.path.join(self._prime_dir, source)
                with contextlib.suppress(FileNotFoundError):
                    os.remove(destination)
                file_utils.link_or_copy(source, destination)

        # Now copy the assets contained within the snap directory directly into
        # meta.
        for origin in ["gui", "hooks"]:
            src_dir = os.path.join("snap", origin)
            dst_dir = os.path.join(self.meta_dir, origin)
            if os.path.isdir(src_dir):
                os.makedirs(dst_dir, exist_ok=True)
                for asset in os.listdir(src_dir):
                    source = os.path.join(src_dir, asset)
                    destination = os.path.join(dst_dir, asset)

                    # First, verify that the hook is actually executable
                    if origin == "hooks":
                        _validate_hook(source)

                    with contextlib.suppress(FileNotFoundError):
                        os.remove(destination)

                    file_utils.link_or_copy(source, destination)

        self._record_manifest_and_source_snapcraft_yaml()

    def generate_hook_wrappers(self) -> None:
        snap_hooks_dir = os.path.join(self._prime_dir, "snap", "hooks")
        hooks_dir = os.path.join(self._prime_dir, "meta", "hooks")
        if os.path.isdir(snap_hooks_dir):
            os.makedirs(hooks_dir, exist_ok=True)
            for hook_name in os.listdir(snap_hooks_dir):
                file_path = os.path.join(snap_hooks_dir, hook_name)
                # First, verify that the hook is actually executable
                if not os.stat(file_path).st_mode & stat.S_IEXEC:
                    raise meta_errors.CommandError(
                        "hook {!r} is not executable".format(hook_name)
                    )

                hook_exec = os.path.join("$SNAP", "snap", "hooks", hook_name)
                hook_path = os.path.join(hooks_dir, hook_name)
                with contextlib.suppress(FileNotFoundError):
                    os.remove(hook_path)

                self._write_wrap_exe(hook_exec, hook_path)

    def _setup_gui(self):
        # Handles the setup directory which only contains gui assets.
        setup_dir = "setup"
        if not os.path.exists(setup_dir):
            return

        handle_deprecation_notice("dn3")

        gui_src = os.path.join(setup_dir, "gui")
        gui_dst = os.path.join(self.meta_dir, "gui")
        if os.path.exists(gui_src):
            for f in os.listdir(gui_src):
                if not os.path.exists(gui_dst):
                    os.mkdir(gui_dst)
                shutil.copy2(os.path.join(gui_src, f), gui_dst)

    def _compose_snap_yaml(self):
        """Create a new dictionary from config_data to obtain snap.yaml.

        Missing key exceptions will be raised if config_data does not contain
        all the _MANDATORY_PACKAGE_KEYS, config_data can be validated against
        the snapcraft schema.

        Keys that are in _OPTIONAL_PACKAGE_KEYS are ignored if not there.
        """
        snap_yaml = collections.OrderedDict()

        for key_name in _MANDATORY_PACKAGE_KEYS:
            snap_yaml[key_name] = self._config_data[key_name]

        # Reparse the version, the order should stick.
        snap_yaml["version"] = _version.get_version(
            self._config_data["version"], self._config_data.get("version-script")
        )

        for key_name in _OPTIONAL_PACKAGE_KEYS:
            if key_name in self._config_data:
                snap_yaml[key_name] = self._config_data[key_name]

        if "apps" in self._config_data:
            _verify_app_paths(basedir=self._prime_dir, apps=self._config_data["apps"])
            snap_yaml["apps"] = self._wrap_apps(self._config_data["apps"])
            self._render_socket_modes(snap_yaml["apps"])
            self._validate_command_chain(snap_yaml["apps"])

        self._process_passthrough_properties(snap_yaml)
        assumes = _determine_assumes(self._config_data)
        if assumes:
            # Sorting for consistent results (order doesn't matter)
            snap_yaml["assumes"] = sorted(set(snap_yaml.get("assumes", [])) | assumes)

        return snap_yaml

    def _write_wrap_exe(self, wrapexec, wrappath, shebang=None, args=None, cwd=None):
        if args:
            quoted_args = ['"{}"'.format(arg) for arg in args]
        else:
            quoted_args = []
        args = " ".join(quoted_args) + ' "$@"' if args else '"$@"'
        cwd = "cd {}".format(cwd) if cwd else ""

        executable = '"{}"'.format(wrapexec)

        if shebang:
            if shebang.startswith("/usr/bin/env "):
                shebang = shell_utils.which(shebang.split()[1])
            new_shebang = self._install_path_pattern.sub("$SNAP", shebang)
            new_shebang = re.sub(self._prime_dir, "$SNAP", new_shebang)
            if new_shebang != shebang:
                # If the shebang was pointing to and executable within the
                # local 'parts' dir, have the wrapper script execute it
                # directly, since we can't use $SNAP in the shebang itself.
                executable = '"{}" "{}"'.format(new_shebang, wrapexec)

        with open(wrappath, "w+") as f:
            print("#!/bin/sh", file=f)
            if cwd:
                print("{}".format(cwd), file=f)
            print("exec {} {}".format(executable, args), file=f)

        os.chmod(wrappath, 0o755)

    def _wrap_exe(self, command, basename=None):
        execparts = shlex.split(command)
        exepath = os.path.join(self._prime_dir, execparts[0])
        if basename:
            wrappath = os.path.join(self._prime_dir, basename) + ".wrapper"
        else:
            wrappath = exepath + ".wrapper"
        shebang = None

        if os.path.exists(wrappath):
            os.remove(wrappath)

        wrapexec = "$SNAP/{}".format(execparts[0])
        if not os.path.exists(exepath) and "/" not in execparts[0]:
            _find_bin(execparts[0], self._prime_dir)
            wrapexec = execparts[0]
        else:
            with open(exepath, "rb") as exefile:
                # If the file has a she-bang, the path might be pointing to
                # the local 'parts' dir. Extract it so that _write_wrap_exe
                # will have a chance to rewrite it.
                if exefile.read(2) == b"#!":
                    shebang = exefile.readline().strip().decode("utf-8")

        self._write_wrap_exe(wrapexec, wrappath, shebang=shebang, args=execparts[1:])

        return os.path.relpath(wrappath, self._prime_dir)

    def _wrap_apps(self, apps: Dict[str, Any]) -> Dict[str, Any]:
        apps = copy.deepcopy(apps)
        gui_dir = os.path.join(self.meta_dir, "gui")
        if not os.path.exists(gui_dir):
            os.mkdir(gui_dir)
        for f in os.listdir(gui_dir):
            if os.path.splitext(f)[1] == ".desktop":
                os.remove(os.path.join(gui_dir, f))
        for app_name, app in apps.items():
            adapter = project_loader.Adapter[app.pop("adapter").upper()]
            if adapter == project_loader.Adapter.LEGACY:
                self._wrap_app(app_name, app)
            elif adapter == project_loader.Adapter.FULL:
                self._add_command_chain_to_app(app_name, app)
            self._generate_desktop_file(app_name, app)
        return apps

    def _add_command_chain_to_app(self, app_name, app):
        # First, validate the commands. Without a wrapper, we're bound by snapd's
        # restrictions. No quotes, and the first string must be a relative path from
        # the root of the snap.
        commands = (app[k] for k in ("command", "stop-command") if k in app)
        for command in commands:
            if not _APP_COMMAND_PATTERN.match(command):
                raise errors.InvalidAppCommandFormatError(command, app_name)

            command_without_args = command.split()[0]
            binary_path = os.path.join(self._prime_dir, command_without_args)
            is_executable = False
            with contextlib.suppress(FileNotFoundError):
                mode = os.stat(binary_path).st_mode
                is_executable = (
                    mode & stat.S_IXUSR or mode & stat.S_IXGRP or mode & stat.S_IXOTH
                )

            if not is_executable:
                raise errors.InvalidAppCommandError(command_without_args, app_name)

        # Finally, assuming there is a meta runner, add it to the BEGINNING of the
        # command chain. This is to ensure all subsequent commands in the chain get
        # a valid environment.
        if os.path.isfile(self._meta_runner):
            app["command-chain"] = [
                os.path.relpath(self._meta_runner, self._prime_dir)
            ] + app.get("command-chain", [])

    def _wrap_app(self, name, app):
        cmds = (k for k in ("command", "stop-command") if k in app)
        for k in cmds:
            try:
                new_command = self._wrap_exe(app[k], "{}-{}".format(k, name))
            except FileNotFoundError:
                raise errors.InvalidAppCommandError(command=app[k], app_name=app)
            except meta_errors.CommandError as e:
                raise errors.InvalidAppCommandError(str(e), name)

            if os.path.isfile(self._meta_runner):
                snap_runner = os.path.relpath(self._meta_runner, self._prime_dir)
                new_command = "{} $SNAP/{}".format(snap_runner, new_command)
            app[k] = new_command

    def _generate_desktop_file(self, name, app):
        desktop_file_name = app.pop("desktop", "")
        if desktop_file_name:
            desktop_file = _desktop.DesktopFile(
                name=name,
                filename=desktop_file_name,
                snap_name=self._config_data["name"],
                prime_dir=self._prime_dir,
            )
            desktop_file.parse_and_reformat()
            desktop_file.write(gui_dir=os.path.join(self.meta_dir, "gui"))

    def _render_socket_modes(self, apps: Dict[str, Any]) -> None:
        for app in apps.values():
            sockets = app.get("sockets", {})
            for socket in sockets.values():
                mode = socket.get("socket-mode")
                if mode is not None:
                    socket["socket-mode"] = OctInt(mode)

    def _validate_command_chain(self, apps: Dict[str, Any]) -> None:
        for app_name, app in apps.items():
            for item in app.get("command-chain", []):
                executable_path = os.path.join(self._prime_dir, item)

                # command-chain entries must always be relative to the root of the snap,
                # i.e. PATH is not used.
                if not _executable_is_valid(executable_path):
                    raise errors.InvalidCommandChainError(item, app_name)

    def _process_passthrough_properties(self, snap_yaml: Dict[str, Any]) -> None:
        passthrough_applied = False

        for section in ["apps", "hooks"]:
            if section in self._config_data:
                for name, value in snap_yaml[section].items():
                    if self._apply_passthrough(
                        value,
                        value.pop("passthrough", {}),
                        self._original_snapcraft_yaml[section][name],
                    ):
                        passthrough_applied = True

        if self._apply_passthrough(
            snap_yaml,
            self._config_data.get("passthrough", {}),
            self._original_snapcraft_yaml,
        ):
            passthrough_applied = True

        if passthrough_applied:
            logger.warn(
                "The 'passthrough' property is being used to "
                "propagate experimental properties to snap.yaml "
                "that have not been validated."
            )

    def _apply_passthrough(
        self,
        section: Dict[str, Any],
        passthrough: Dict[str, Any],
        original: collections.OrderedDict,
    ) -> bool:
        # Any value already in the original dictionary must
        # not be specified in passthrough at the same time.
        duplicates = list(original.keys() & passthrough.keys())
        if duplicates:
            raise meta_errors.AmbiguousPassthroughKeyError(duplicates)
        section.update(passthrough)
        return bool(passthrough)


def _determine_assumes(yaml_data: Dict[str, Any]) -> Set[str]:
    # Order doesn't really matter, but we don't want duplicates, so use a set
    assumes = set()

    # Check to see if any apps use the "full" adapter, which requires command-chain
    for app in yaml_data.get("apps", dict()).values():
        adapter = project_loader.Adapter[app["adapter"].upper()]
        if adapter == project_loader.Adapter.FULL:
            assumes.add("command-chain")

    return assumes


def _find_bin(binary, basedir):
    # If it doesn't exist it might be in the path
    logger.debug("Checking that {!r} is in the $PATH".format(binary))
    try:
        shell_utils.which(binary, cwd=basedir)
    except subprocess.CalledProcessError:
        raise meta_errors.CommandError(binary)


def _validate_hook(hook_path):
    if not os.stat(hook_path).st_mode & stat.S_IEXEC:
        asset = os.path.basename(hook_path)
        raise meta_errors.CommandError("hook {!r} is not executable".format(asset))


def _verify_app_paths(basedir, apps):
    for app in apps:
        path_entries = [i for i in ("desktop", "completer") if i in apps[app]]
        for path_entry in path_entries:
            file_path = os.path.join(basedir, apps[app][path_entry])
            if not os.path.exists(file_path):
                raise errors.SnapcraftPathEntryError(
                    app=app, key=path_entry, value=file_path
                )


def _executable_is_valid(path: str) -> bool:
    with contextlib.suppress(FileNotFoundError):
        mode = os.stat(path).st_mode
        return bool(mode & stat.S_IXUSR or mode & stat.S_IXGRP or mode & stat.S_IXOTH)

    return False
