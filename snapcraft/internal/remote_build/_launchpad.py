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

import datetime
import logging
import os
import shutil
import time
import urllib.request
import urllib.error
import urllib.parse

from lazr import restfulclient
from lazr.restfulclient.resource import Entry
from launchpadlib.launchpad import Launchpad
from typing import Any, Dict, List, Sequence, Tuple, Optional
from xdg import BaseDirectory
from . import errors

import snapcraft
from snapcraft.internal.sources.errors import SnapcraftPullError
from snapcraft.internal.sources._git import Git
from snapcraft.project import Project

_LP_POLL_INTERVAL = 30
_LP_SUCCESS_STATUS = "Successfully built"
_LP_FAIL_STATUS = "Failed to build"

logger = logging.getLogger(__name__)


class LaunchpadClient:
    """Launchpad remote builder operations."""

    def __init__(
        self,
        *,
        project: Project,
        build_id: str,
        user: str,
        architectures: Optional[List[str]] = None,
        git_branch: str = "master",
        core18_channel: str = "stable",
        snapcraft_channel: str = "edge",
    ) -> None:
        if not Git.check_command_installed():
            raise errors.GitNotFoundProviderError(provider="Launchpad")

        self._snap_name = project.info.name
        self._build_id = build_id

        self.architectures = architectures
        self.user = user

        self._lp_name = build_id
        self._lp_git_branch = git_branch

        self._core18_channel = core18_channel
        self._snapcraft_channel = snapcraft_channel

        self._cache_dir = self._create_cache_directory()
        self._data_dir = self._create_data_directory()
        self._credentials = os.path.join(self._data_dir, "credentials")

        self._lp: Launchpad = None
        self._waiting = []  # type: List[str]

    @property
    def architectures(self) -> Sequence[str]:
        return self._architectures

    @architectures.setter
    def architectures(self, architectures: Sequence[str]) -> None:
        if architectures:
            self._lp_processors = ["/+processors/" + a for a in architectures]
        else:
            self._lp_processors = None

        self._architectures = architectures

    @property
    def user(self) -> str:
        return self._lp_user

    @user.setter
    def user(self, user: str) -> None:
        self._lp_user = user
        self._lp_owner = f"/~{user}"

    def _create_data_directory(self) -> str:
        data_dir = BaseDirectory.save_data_path("snapcraft", "provider", "launchpad")
        os.makedirs(data_dir, mode=0o700, exist_ok=True)
        return data_dir

    def _create_cache_directory(self) -> str:
        cache_dir = BaseDirectory.save_cache_path("snapcraft", "provider", "launchpad")
        os.makedirs(cache_dir, mode=0o700, exist_ok=True)
        return cache_dir

    def login(self) -> Launchpad:
        self._lp = Launchpad.login_with(
            "snapcraft remote-build {}".format(snapcraft.__version__),
            "production",
            self._cache_dir,
            credentials_file=self._credentials,
            version="devel",
        )

    def get_git_repo_path(self) -> str:
        return f"~{self._lp_user}/+git/{self._lp_name}"

    def get_git_https_url(self, token: Optional[str] = None) -> str:
        if token:
            return f"https://{self._lp_user}:{token}@git.launchpad.net/~{self._lp_user}/+git/{self._lp_name}/"
        else:
            return f"https://{self._lp_user}@git.launchpad.net/~{self._lp_user}/+git/{self._lp_name}/"

    def create_git_repository(self, force=False) -> Entry:
        """Create git repository."""
        if force:
            self.delete_git_repository()

        logger.debug(
            f"creating git repo: name={self._lp_name}, owner={self._lp_owner}, target={self._lp_owner}"
        )
        return self._lp.git_repositories.new(
            name=self._lp_name, owner=self._lp_owner, target=self._lp_owner
        )

    def delete_git_repository(self) -> None:
        """Delete git repository."""
        git_path = self.get_git_repo_path()
        git_repo = self._lp.git_repositories.getByPath(path=git_path)

        # git_repositories.getByPath returns None if git repo does not exist.
        if git_repo is None:
            return

        git_repo.lp_delete()

    def create_snap(self, force=False) -> Entry:
        """Create a snap recipe. Use force=true to replace existing snap."""
        git_url = self.get_git_https_url()

        if force:
            self.delete_snap()

        optional_kwargs = dict()
        if self._lp_processors:
            optional_kwargs["processors"] = self._lp_processors

        return self._lp.snaps.new(
            name=self._lp_name,
            owner=self._lp_owner,
            git_repository_url=git_url,
            git_path=self._lp_git_branch,
            auto_build=False,
            auto_build_archive="/ubuntu/+archive/primary",
            auto_build_pocket="Updates",
            **optional_kwargs,
        )

    def delete_snap(self) -> None:
        """Remove a snap recipe and all associated files."""
        try:
            # snaps.getByName raises NotFound if snap not does not exist.
            lp_snap = self._lp.snaps.getByName(name=self._lp_name, owner=self._lp_owner)
        except restfulclient.errors.NotFound:
            return

        lp_snap.lp_delete()

    def cleanup(self) -> None:
        """Delete snap and git repository from launchpad."""
        self.delete_snap()
        self.delete_git_repository()

    def start_build(self, timeout: int = 5, attempts: int = 5) -> int:
        """Initiate a new snap build."""
        snap = self.create_snap(force=True)
        dist = self._lp.distributions["ubuntu"]

        snap_build_request = snap.requestBuilds(
            archive=dist.main_archive,
            channels={
                "core18": self._core18_channel,
                "snapcraft": self._snapcraft_channel,
            },
            pocket="Updates",
        )
        logger.debug("Request URL: {}".format(snap_build_request))

        while attempts > 0 and snap_build_request.status == "Pending":
            logger.debug(
                "Build request: status={} error={}".format(
                    snap_build_request.status, snap_build_request.error_message
                )
            )

            time.sleep(timeout)

            # Refresh status.
            snap_build_request.lp_refresh()
            attempts -= 1

        # Build request failed.
        if snap_build_request.status == "Failed":
            self.cleanup()
            raise errors.RemoteBuilderError(
                builder_error=snap_build_request.error_message
            )

        # Timed out.
        if snap_build_request.status == "Pending":
            self.cleanup()
            raise errors.RemoteBuilderNotReadyError()

        # Shouldn't end up here.
        if snap_build_request.status != "Completed":
            self.cleanup()
            raise errors.RemoteBuilderError(
                builder_error="Unknown builder error - reported status: {}".format(
                    snap_build_request.status
                )
            )

        # Shouldn't end up here either.
        if not snap_build_request.builds.entries:
            self.cleanup()
            raise errors.RemoteBuilderError(
                builder_error="Unknown builder error - no build entries found."
            )

        self._waiting = [
            build["arch_tag"] for build in snap_build_request.builds.entries
        ]
        self._builds_collection_link = snap_build_request.builds_collection_link
        build_number = snap_build_request.self_link.rsplit("/", 1)[-1]
        return build_number

    def recover_build(self, req_number: int) -> None:
        """Prepare internal state to monitor an existing build."""
        url = "https://api.launchpad.net/devel/~{}/+snap/{}/+build-request/{}".format(
            self._lp_user, self._lp_name, req_number
        )

        try:
            request = self._lp.load(url)
        except restfulclient.errors.NotFound:
            raise errors.RemoteBuildNotFoundError(
                name=self._snap_name, req_number=req_number
            )

        logger.debug("Request URL: {}".format(request))
        builds = self._lp.load(request.builds_collection_link)
        self._waiting = [build["arch_tag"] for build in builds.entries]
        self._builds_collection_link = request.builds_collection_link

    def monitor_build(self, interval: int = _LP_POLL_INTERVAL) -> None:
        """Check build progress, and download artifacts when ready."""
        logger.debug("Monitoring builds: {}".format(" ".join(self._waiting)))
        while len(self._waiting):
            time.sleep(interval)
            builds = self._lp.load(self._builds_collection_link)
            if not builds.entries:
                break

            for build in builds.entries:
                arch = build["arch_tag"]
                build_state = build["buildstate"]
                logger.debug("{} state: {}".format(arch, build_state))
                if arch in self._waiting:
                    if build_state == _LP_SUCCESS_STATUS:
                        self._process_success(build)
                    elif build_state == _LP_FAIL_STATUS:
                        self._process_fail(build)

    def get_build_status(self) -> List[Tuple[str, str]]:
        status = []  # type: List[Tuple[str, str]]
        builds = self._lp.load(self._builds_collection_link)
        for build in builds.entries:
            arch = build["arch_tag"]
            build_state = build["buildstate"]
            status.append((arch, build_state))

        return status

    def _process_success(self, build: Dict[str, Any]) -> None:
        arch = build["arch_tag"]
        snap_build = self._lp.load(build["self_link"])
        urls = snap_build.getFileUrls()
        logger.debug("Success urls: {}".format(urls))
        if not urls:
            logger.error("Snap file not available for arch {}.".format(arch))
            self._waiting.remove(arch)
            return

        self._download_log(build)

        for url in urls:
            snap_name = url.rsplit("/", 1)[-1]
            # Sanity check
            if not snap_name.endswith("_{}.snap".format(arch)):
                continue
            try:
                self._download_file(url, snap_name)
                logger.info("Snapped {}".format(snap_name))
            except urllib.error.HTTPError as e:
                logger.error("Snap download error: {}: {}".format(e, snap_name))

        self._waiting.remove(arch)

    def _process_fail(self, build: Dict[str, Any]) -> None:
        try:
            arch = build["arch_tag"]
            log_name = self._download_log(build)
            logger.error(
                "Build failed for arch {}. Log file is {!r}.".format(arch, log_name)
            )
        except urllib.error.HTTPError as e:
            logger.error("Log file download error ({}): {}".format(arch, e))
        finally:
            self._waiting.remove(arch)

    def _get_logfile_name(self, build: Dict[str, Any]) -> str:
        arch = build["arch_tag"]
        log_name = name = "{}_{}.txt.gz".format(self._snap_name, arch)
        number = 1
        while os.path.isfile(log_name):
            log_name = "{}.{}".format(name, number)
            number += 1
        return log_name

    def _download_log(self, build: Dict[str, Any]) -> str:
        url = build["build_log_url"]
        logger.debug("Build log url: {}".format(url))
        log_name = self._get_logfile_name(build)
        self._download_file(url, log_name)
        return log_name

    def _download_file(self, url: str, name: str) -> None:
        logger.debug("Download snap from {!r}".format(url))
        with urllib.request.urlopen(url) as response, open(name, "wb") as snapfile:
            shutil.copyfileobj(response, snapfile)  # type: ignore

    def _gitify_repository(self, repo_dir: str) -> Git:
        """Git-ify source repository tree.

        :return: Git handler instance to git repository.
        """
        git_handler = Git(repo_dir, repo_dir, silent=True)

        # Init repo.
        git_handler.init()

        # Add all files found in repo.
        for f in os.listdir(repo_dir):
            if f != ".git":
                git_handler.add(f)

        # Commit files.
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
        git_handler.commit(
            "snapcraft commit\n\nversion: {}\ntimestamp: {}\n".format(
                snapcraft.__version__, timestamp
            )
        )

        return git_handler

    def push_source_tree(self, repo_dir: str) -> str:
        """Push source tree to launchpad, returning URL."""
        git_handler = self._gitify_repository(repo_dir)
        lp_repo = self.create_git_repository(force=True)
        token = lp_repo.issueAccessToken()

        url = self.get_git_https_url(token=token)
        stripped_url = self.get_git_https_url(token="<token>")

        logger.info(f"Sending data to remote builder... ({stripped_url})")

        try:
            git_handler.push(url, f"HEAD:{self._lp_git_branch}", force=True)
        except SnapcraftPullError as error:
            # Strip token from command.
            command = error.command.replace(token, "<token>")  # type: ignore
            exit_code = error.exit_code  # type: ignore
            raise errors.LaunchpadGitPushError(command=command, exit_code=exit_code)

        return url
