# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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
import pathlib
import shutil
import subprocess
from collections import OrderedDict
from copy import deepcopy

import snapcraft
import snapcraft.internal.sources
from snapcraft import yaml_utils
from snapcraft.file_utils import rmtree
from snapcraft.internal.meta import _version
from snapcraft.internal.remote_build import errors
from snapcraft.project import Project

logger = logging.getLogger(__name__)


class WorkTree:
    """Manages tree for remote-build project."""

    def __init__(
        self, worktree_dir: str, project: Project, package_all_sources=False
    ) -> None:
        """Create remote-build WorkTree.

        :param str worktree_dir: Directory to use for working tree.
        :param Project project: Snapcraft project.
        :param bool package_all_sources: Package all sources instead of
               the default of local-only sources.
        """
        self._project = project
        self._package_all_sources = package_all_sources

        # Get snapcraft yaml (as OrderedDict).
        self._snapcraft_config = (
            self._project.info.get_raw_snapcraft()
        )  # type: OrderedDict

        # Working copy.
        self._prepared_snapcraft_config = (
            self._project.info.get_raw_snapcraft()
        )  # type: OrderedDict

        # Working tree base directory.
        self._base_dir = worktree_dir

        # Initialize cache for sources.
        self._cache_dir = os.path.join(self._base_dir, "cache")
        os.makedirs(self._cache_dir, exist_ok=True)

        # Initialize clean repo to ship to remote builder.
        self._repo_dir = os.path.join(self._base_dir, "repo")
        if os.path.exists(self._repo_dir):
            rmtree(self._repo_dir)

        self._repo_sources_dir = os.path.join(self._repo_dir, "sources")
        self._repo_snap_dir = os.path.join(self._repo_dir, "snap")

    def _get_part_source_handler(self, part_name: str, source: str, source_dir: str):
        """Get an instance for a part's source handler.

        :param str part_name: Name of part.
        :param str source: Source URL.
        :param str source_dir: Directory to pull sources to.
        :return: Source handler instanced configured with source and source_dir.
        """

        # The actual source may be a selector, override it.
        part_config = self._snapcraft_config["parts"][part_name]
        part_config["source"] = source
        source_type = part_config.get("source-type")

        handler_class = snapcraft.internal.sources.get_source_handler(
            source, source_type=source_type
        )
        return handler_class(
            source,
            source_dir,
            source_checksum=part_config.get("source-checksum"),
            source_branch=part_config.get("source-branch"),
            source_tag=part_config.get("source-tag"),
            source_depth=part_config.get("source-depth"),
            source_commit=part_config.get("source-commit"),
        )

    def _get_part_cache_dir(self, part_name: str, selector=None) -> str:
        """Get source cache directory for part_name.

        Takes into account source selector, if provided, as parts using source
        selectors will require multiple source directories.

        :param str part_name: Name of part.
        :param str selector: Source selector, if any (e.g. "on amd64").
        :return: Directory path to store source.
        """
        if selector:
            return os.path.join(self._cache_dir, part_name, selector)

        return os.path.join(self._cache_dir, part_name)

    def _get_part_tarball_path(self, part_name: str, selector=None) -> str:
        """Get path to archive the specified part's source tarball in the repo.

        :param str part_name: Name of part.
        :param str selector: Source selector, if any (e.g. "on amd64").
        :return: File path for tarball.
        """
        tarball_filename = part_name + ".tar.gz"
        if selector:
            return os.path.join(
                self._repo_sources_dir, part_name, selector, tarball_filename
            )
        return os.path.join(self._repo_sources_dir, part_name, tarball_filename)

    def _archive_part_sources(self, part_name: str, selector=None):
        """Archive sources at source_dir into archive_path.

        :param str part_name: Name of part.
        :param str selector: Source selector, if any (e.g. "on amd64").
        :return: Relative path to archive within repository.
        """
        source_path = self._get_part_cache_dir(part_name, selector)
        archive_path = self._get_part_tarball_path(part_name, selector)

        logger.debug("creating source archive: {}".format(archive_path))

        os.makedirs(os.path.split(archive_path)[0], exist_ok=True)
        command = ["tar", "czf", archive_path, "-C", source_path, "."]
        subprocess.check_call(command)
        relpath = os.path.relpath(archive_path, self._repo_dir)

        # Ensure Linux path formatting is used.
        return pathlib.Path(relpath).as_posix()

    def _pull_source(self, part_name: str, source: str, selector=None) -> str:
        """Pull source_url for part to source_dir. Returns source.

        :param str part_name: Name of part.
        :param str source: Source URL to pull.
        :param str selector: Source selector, if any (e.g. "on amd64").
        :return: Relative path to archive within repository.
        """
        download_dir = self._get_part_cache_dir(part_name, selector)
        source_handler = self._get_part_source_handler(part_name, source, download_dir)

        if selector:
            print_name = "{} [selector='{}']".format(part_name, selector)
        else:
            print_name = part_name

        # Skip non-local sources (the remote builder can fetch those directly),
        # unless configured to package all sources.
        is_local_source = isinstance(
            source_handler, snapcraft.internal.sources.Local
        ) or (
            isinstance(source_handler, snapcraft.internal.sources.Git)
            and source_handler.is_local()
        )
        if not self._package_all_sources and not is_local_source:
            logger.debug("passing through source for {}: {}".format(print_name, source))
            return source

        logger.info("Packaging sources for {}...".format(print_name))

        # Remove existing cache directory (if exists)
        if os.path.exists(download_dir):
            rmtree(download_dir)

        # Pull sources, but create directory first if part
        # is configured to use the `dump` plugin.
        if self._snapcraft_config["parts"][part_name].get("plugin") == "dump":
            os.makedirs(download_dir, exist_ok=True)

        source_handler.pull()

        # Create source archive.
        return self._archive_part_sources(part_name, selector)

    def _process_part_sources(
        self, part_name: str, part_config: OrderedDict
    ) -> OrderedDict:
        """Add part sources to tree.

        :param str part_name: Name of part.
        :param OrderedDict part_config: Part configuration for specified part.
        :return: Modified part configuration for prepared snapcraft.yaml.
        """
        logger.debug("processing {}: {}".format(part_name, part_config))

        # We will be modifying part_config, copy it.
        part_config = deepcopy(part_config)

        if "source" not in part_config:
            logger.debug("no source specified, skipping...")
            return part_config

        source = part_config["source"]
        source_modified = False
        if isinstance(source, list):
            # Handle advanced grammar case for `source` by parsing an
            # array of {<selector>: <source>} OrderedDicts. Example:
            # source:
            # - on amd64: http://archive.ubuntu.com/foo_amd64.deb
            # - on arm64: http://archive.ubuntu.com/foo_arm64.deb
            # - on i386: http://archive.ubuntu.com/foo_i386.deb
            # Results in:
            # [
            #   {'on amd64': 'http://archive.ubuntu.com/foo_amd64.deb'},
            #   {'on arm64': 'http://archive.ubuntu.com/foo_arm64.deb'},
            #   {'on i386': 'http://archive.ubuntu.com/foo_i386.deb'},
            # ]
            part_config["source"] = []
            for s in source:
                sinfo = OrderedDict()  # type: OrderedDict
                # Generally speaking, there will only be one key in each
                # dict (e.g. "on amd64"), but we iterate regardless.
                for selector, selector_source in s.items():
                    sinfo[selector] = self._pull_source(
                        part_name, selector_source, selector=selector
                    )
                    if sinfo[selector] != selector_source:
                        source_modified = True

                part_config["source"].append(sinfo)
        else:
            part_config["source"] = self._pull_source(part_name, source)
            if part_config["source"] != source:
                source_modified = True

        if source_modified:
            # Strip source attributes no longer relevant.
            for key in list(part_config.keys()):
                if key.startswith("source-") and key != "source-subdir":
                    part_config.pop(key)

            # It's now an archive, set it explicitly.
            part_config["source-type"] = "tar"

        return part_config

    def _copy_assets(self):
        """Copy assets directly to repository, if exists."""
        assets_dir = self._project._get_snapcraft_assets_dir()
        logger.debug("assets expected at: {}".format(assets_dir))

        # Assets directory exists, but may not actually contain assets.
        # The snapd project is an example of this.  The sanity checker
        # will warn to the user when the project is built.
        if os.path.exists(assets_dir):
            logger.debug(
                "copying assets: {} -> {}".format(assets_dir, self._repo_snap_dir)
            )

            # Remove existing assets, and update copy.
            if os.path.exists(self._repo_snap_dir):
                rmtree(self._repo_snap_dir)
            shutil.copytree(assets_dir, self._repo_snap_dir)
        else:
            os.makedirs(self._repo_snap_dir, exist_ok=True)

        # Copy plugins if not already inside assets directory (and exists).
        plugins_dir = self._project._get_local_plugins_dir()
        if plugins_dir.startswith(assets_dir) or not os.path.exists(plugins_dir):
            return

        self._repo_plugins_dir = os.path.join(self._repo_snap_dir, "plugins")
        logger.debug(
            "copying local plugins: {} -> {}".format(
                plugins_dir, self._repo_plugins_dir
            )
        )
        shutil.copytree(plugins_dir, self._repo_plugins_dir)

    def _set_prepared_project_version(self):
        """Set project version to handle `version: git`
        This and version-script expect to run in project root (".").
        """
        # version-script is deprecated, remote-build won't support it.
        if "version-script" in self._snapcraft_config:
            raise errors.UnsupportedVersionScriptError()

        version = self._snapcraft_config.get("version")

        # Nothing to do here if no version is set.
        if version is None:
            return

        # Determine git version, if `version: git`.
        if version == "git":
            try:
                version = _version.get_version(version, version_script=None)
            except FileNotFoundError:
                raise errors.GitNotFoundVersionError()

        self._prepared_snapcraft_config["version"] = version

    def prepare_repository(self) -> str:
        """Prepare source tree for launchpad build. Returns repo directory."""
        # Copy project assets.
        self._copy_assets()

        # Create sources directory for source archives.
        os.makedirs(self._repo_sources_dir, exist_ok=True)

        # Process each part with sources.
        for part_name, part_config in self._snapcraft_config["parts"].items():
            part_config = self._process_part_sources(part_name, part_config)
            self._prepared_snapcraft_config["parts"][part_name] = part_config

        # Set version.
        self._set_prepared_project_version()

        # Write updated snapcraft yaml config.
        snapcraft_yaml_path = os.path.join(self._repo_snap_dir, "snapcraft.yaml")
        with open(snapcraft_yaml_path, "w") as f:
            yaml_utils.dump(self._prepared_snapcraft_config, stream=f)

        return self._repo_dir
